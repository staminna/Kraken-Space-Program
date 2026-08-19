//! Origin shifting — keeping the active vessel near the physics origin.
//!
//! design.md: "f32 precision degrades past ~10,000 units from origin — irrelevant at human
//! scale, fatal at planetary scale. Implement this in Phase 1, not as a later patch."
//!
//! # What actually moves
//!
//! Rapier simulates in a local f32 frame. When the active vessel drifts too far from that
//! frame's origin, everything in the world is translated back by the same delta and
//! [`WorldOrigin`] absorbs it:
//!
//! ```text
//!   WorldOrigin += delta          (f64, gains the offset)
//!   every Rapier transform -= delta   (f32, loses it)
//!   SimPosition = WorldOrigin + transform      →  unchanged
//! ```
//!
//! Because the two changes cancel exactly in f64, no simulation position moves. Nothing
//! outside this file needs to know a shift happened — which is the whole point, and why
//! the shift has to be one atomic system pass rather than something systems opt into.
//!
//! # Why LocalOrigin tracks WorldOrigin exactly
//!
//! [`LocalOrigin`] is what `render_sync` subtracts, so render space is
//! `SimPosition - LocalOrigin`. Setting it equal to `WorldOrigin` makes render space
//! identical to Rapier's local frame — which means static scenery that Rapier owns
//! directly (terrain colliders, the launch pad) and interpolated vessels stay in the same
//! space without the scenery needing its own f64 position. They are kept as separate
//! resources because design.md anticipates them diverging later, when the render origin
//! follows the camera rather than the physics frame.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::rendering::render_sync::{LocalOrigin, WorldOrigin};
use crate::vessel::components::{ActiveVessel, VesselId};

/// How far the active vessel may drift from the physics origin before everything shifts.
///
/// design.md's default. At 10 km an f32 has ~1 mm of precision, which is still far finer
/// than any collision this engine resolves.
pub const ORIGIN_SHIFT_THRESHOLD_M: f32 = 10_000.0;

/// Shifts the world origin when the active vessel drifts past the threshold.
///
/// # Ordering
///
/// `FixedUpdate`, after `PhysicsSet::Writeback` and after the readback that derives
/// `SimPosition`, so it sees settled positions and its correction lands before the next
/// tick's `SyncBackend` reads transforms back into Rapier.
pub fn shift_world_origin(
    mut world_origin: ResMut<WorldOrigin>,
    mut local_origin: ResMut<LocalOrigin>,
    active_vessels: Query<Entity, With<ActiveVessel>>,
    // One query, iterated twice: reading `&Transform` in a second query while this one
    // holds `&mut Transform` is a access conflict Bevy rejects at run time, and splitting
    // it with `Without` would exclude exactly the bodies that need shifting.
    mut bodies: Query<(&mut Transform, Option<&VesselId>), With<RigidBody>>,
) {
    let Ok(active) = active_vessels.single() else {
        return;
    };

    // Reference point: the mean of the active vessel's parts, so a long stack shifts
    // around its middle instead of around whichever part happened to be the root.
    let mut sum = Vec3::ZERO;
    let mut count = 0.0f32;
    for (transform, vessel_id) in &bodies {
        if vessel_id.is_some_and(|id| id.0 == active) {
            sum += transform.translation;
            count += 1.0;
        }
    }
    if count == 0.0 {
        return;
    }
    let centre = sum / count;

    if centre.length() < ORIGIN_SHIFT_THRESHOLD_M {
        return;
    }

    // Widening f32 → f64 is lossless; this is the direction that is always safe.
    let delta = centre.as_dvec3();
    world_origin.0 += delta;
    local_origin.0 = world_origin.0;

    for (mut transform, _) in &mut bodies {
        transform.translation -= centre;
    }

    // SimPosition is WorldOrigin + local, and both sides just moved by the same amount, so
    // every simulation position is unchanged and nothing needs rewriting. PreviousSimPosition
    // is equally unaffected, which is why interpolation does not jump on the frame a shift
    // happens — that would otherwise be a 10 km visual snap once per shift.
    info!(
        "Krakensbane: origin shifted by {:.1} m, world origin now {:.1?}",
        delta.length(),
        world_origin.0
    );
}
