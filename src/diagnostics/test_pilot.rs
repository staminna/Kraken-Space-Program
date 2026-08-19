//! A scripted pilot, so flight behaviour can be verified without a human at the keyboard.
//!
//! ```text
//! KRAKEN_PILOT=hop KRAKEN_TRACE=all cargo run
//! ```
//!
//! # Why this is permanent rather than a throwaway
//!
//! It was written from scratch three times during Phase 1 and deleted each time, and every
//! rewrite re-introduced the same mistakes — a sign-flipped descent controller once, a
//! forgotten throttle cut at touchdown another time, both of which briefly looked like
//! engine bugs. Landing is the hardest thing in the game to verify by hand and the easiest
//! to break by accident, so the fixture for it belongs in the repository.
//!
//! This is deliberately not a *good* autopilot and is not a game feature. It flies the
//! specific profiles that exercise the physics, using exactly the controls a player has:
//! it writes throttle and pitch/yaw into [`ControlState`] and nothing else.

use bevy::prelude::*;

use crate::celestial::body::CelestialBody;
use crate::part_modules::engine::Engine;
use crate::part_modules::resource_container::{self, ResourceContainer};
use crate::rendering::render_sync::{SimPosition, SimRotation, SimVelocity};
use crate::vessel::components::{
    ActiveVessel, ControlState, Destroyed, PartMass, RootPart, VesselId,
};

/// Gain on the pointing error, in control deflection per radian of tilt.
const POINTING_GAIN: f64 = 3.0;
/// Damping on the body rate, so the attitude hold does not overshoot and ring.
const POINTING_DAMPING: f64 = 2.0;

/// Attitude bias used by `KRAKEN_PILOT_DRIFT` to acquire lateral velocity on the way up.
const DRIFT_BIAS: f64 = 0.35;

/// How far the descent leans into lateral drift, in radians of tilt per m/s.
const DRIFT_CANCEL_GAIN: f64 = 0.06;

/// Furthest from vertical the descent will lean to kill drift, radians (~17°).
///
/// Without a cap the aim point swings past horizontal on the first fast sideways moment and
/// the vessel thrusts itself into the ground.
const MAX_DESCENT_TILT_RAD: f64 = 0.3;

/// Apex the `hop` profile climbs to before turning around, metres.
const HOP_APEX_M: f64 = 60.0;
/// Apex the `ballistic` profile burns to, metres.
const BALLISTIC_APEX_M: f64 = 6_000.0;

/// Root-part altitude below which the vessel counts as down, metres. The root is the top of
/// the stack, so this is the height of the stack itself plus a little.
const TOUCHDOWN_ALTITUDE_M: f64 = 6.0;

#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq)]
pub enum Profile {
    /// Sit still. Useful for watching a vessel settle onto the pad.
    Idle,
    /// Up to [`HOP_APEX_M`], then a controlled descent to a soft landing.
    Hop,
    /// Burn to [`BALLISTIC_APEX_M`] and let it fall back. Exercises drag and impact damage.
    Ballistic,
}

impl Profile {
    pub fn from_env() -> Option<Self> {
        match std::env::var("KRAKEN_PILOT").ok()?.as_str() {
            "idle" => Some(Self::Idle),
            "hop" => Some(Self::Hop),
            "ballistic" => Some(Self::Ballistic),
            other => {
                error!("unknown KRAKEN_PILOT profile '{other}' — expected idle, hop or ballistic");
                None
            }
        }
    }
}

/// The vessel the pilot is allowed to fly: the active one, while it is still in one piece.
type FlyableVessel<'w, 's> =
    Query<'w, 's, (Entity, &'static mut ControlState), (With<ActiveVessel>, Without<Destroyed>)>;

/// The root part's full state, which is everything the pilot steers by.
type RootState<'w, 's> = Query<
    'w,
    's,
    (
        &'static VesselId,
        &'static SimPosition,
        &'static SimVelocity,
        &'static SimRotation,
        &'static bevy_rapier3d::prelude::Velocity,
    ),
    With<RootPart>,
>;

#[derive(Resource, Default)]
pub struct PilotState {
    apex_m: f64,
    landed: bool,
}

/// Flies the active vessel through the configured profile.
pub fn fly(
    profile: Res<Profile>,
    mut state: ResMut<PilotState>,
    body: Res<CelestialBody>,
    mut vessels: FlyableVessel,
    roots: RootState,
    engines: Query<(&VesselId, &Engine)>,
    masses: Query<(&VesselId, &PartMass, Option<&ResourceContainer>)>,
) {
    let Ok((vessel, mut control)) = vessels.single_mut() else {
        return;
    };
    let Some((_, position, velocity, rotation, angular)) =
        roots.iter().find(|(id, ..)| id.0 == vessel)
    else {
        return;
    };

    let altitude = body.altitude_of(position.0);
    let up = body.up_at(position.0);
    let vertical = velocity.0.dot(up);
    state.apex_m = state.apex_m.max(altitude);

    // Where to point. Straight up while climbing; while descending, leaned into the lateral
    // drift so the same burn that slows the fall also kills the sideways motion. A vessel
    // that arrives moving sideways slides, digs a leg in and tips over — which is a real
    // failure, but not the one a landing test is trying to measure.
    let horizontal = velocity.0 - up * vertical;
    let aim = if vertical < 0.0 {
        let lean = (-horizontal * DRIFT_CANCEL_GAIN).clamp_length_max(MAX_DESCENT_TILT_RAD);
        (up + lean).normalize_or_zero()
    } else {
        up
    };

    // Hold the stack pointing at `aim`, using the player's own pitch/yaw controls.
    // Proportional on the pointing error, derivative on the body rate: manual input overrides
    // SAS entirely, so this has to supply its own damping.
    let inverse = rotation.0.inverse();
    let target = (inverse * aim).normalize_or_zero();
    let rate = inverse * angular.angvel.as_dvec3();
    control.pitch = (POINTING_GAIN * target.z - POINTING_DAMPING * rate.x).clamp(-1.0, 1.0) as f32;
    control.yaw = (-POINTING_GAIN * target.x - POINTING_DAMPING * rate.z).clamp(-1.0, 1.0) as f32;
    control.roll = 0.0;

    let max_thrust_n: f64 = engines
        .iter()
        .filter(|(id, _)| id.0 == vessel)
        .map(|(_, engine)| engine.thrust_n)
        .sum();
    let mass_kg = resource_container::vessel_mass_kg(vessel, &masses);
    let gravity = body.gravity_at(position.0).length();

    control.throttle = match *profile {
        Profile::Idle => 0.0,
        Profile::Ballistic => {
            if vertical >= 0.0 && state.apex_m < BALLISTIC_APEX_M {
                1.0
            } else {
                0.0
            }
        }
        Profile::Hop => {
            // Cut at touchdown and stay cut. Without this the engine keeps pushing against
            // the ground, tips the vessel over and drives it across the pad — which took an
            // embarrassingly long time to recognise as the autopilot's fault and not the
            // impact model's.
            if state.landed
                || (state.apex_m > HOP_APEX_M / 2.0
                    && altitude < TOUCHDOWN_ALTITUDE_M
                    && vertical > -1.5)
            {
                state.landed = true;
                0.0
            } else if state.apex_m < HOP_APEX_M && vertical >= 0.0 {
                1.0
            } else if vertical > -0.5 {
                0.0
            } else {
                // Descend faster when high, slow to a crawl near the ground, and hold that
                // profile with a proportional term on top of the hover throttle.
                let wanted = -(1.0 + 0.12 * altitude.max(0.0)).min(30.0);
                let hover = (mass_kg * gravity / max_thrust_n) as f32;
                (hover + ((wanted - vertical) as f32) * 0.06).clamp(0.0, 1.0)
            }
        }
    };

    // Lean over briefly on the way up, so the landing has sideways drift to cope with — a
    // dead vertical hop never tests whether the legs do anything. A *bias* on top of the
    // attitude hold rather than a replacement for it: full deflection here tumbled the
    // stack hard enough to break a joint, which tested something else entirely.
    if matches!(*profile, Profile::Hop)
        && std::env::var("KRAKEN_PILOT_DRIFT").is_ok()
        && (3.0..9.0).contains(&altitude)
        && vertical > 0.0
    {
        control.pitch = (f64::from(control.pitch) + DRIFT_BIAS).clamp(-1.0, 1.0) as f32;
    }
}
