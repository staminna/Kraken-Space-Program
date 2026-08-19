//! Atmosphere, and the drag it exerts.
//!
//! # The Phase 1 model
//!
//! DESIGN.md wants pressure and temperature *curves* defined per body in Lua. This is the
//! flat model it says to build first: a single exponential, which is what an isothermal
//! atmosphere in hydrostatic equilibrium actually is, and which is within a few percent of
//! the real thing up to the altitudes a Phase 1 rocket reaches.
//!
//! ```text
//!   ρ(h) = ρ₀ · exp(-h / H)
//! ```
//!
//! Replacing it with sampled curves later changes [`Atmosphere::density_at`] and nothing
//! else — every consumer already goes through it.

use bevy::prelude::*;

use crate::celestial::body::CelestialBody;
use crate::physics::forces::PendingForces;
use crate::rendering::render_sync::{SimPosition, SimVelocity};

/// A body's atmosphere.
#[derive(Resource, Debug, Clone, Copy)]
pub struct Atmosphere {
    /// Altitude at which the atmosphere ends, metres. Above this, density is exactly zero.
    ///
    /// An exponential never actually reaches zero, and a vessel in a 200 km orbit should not
    /// be losing energy to a millionth of a pascal for the rest of the save. A hard ceiling
    /// is also what "not in atmosphere" means for the on-rails transition in DESIGN.md.
    pub height_m: f64,
    /// Density at the surface, kg/m³.
    pub sea_level_density: f64,
    /// Pressure at the surface, pascals.
    pub sea_level_pressure_pa: f64,
    /// The altitude over which density falls by a factor of e, metres.
    pub scale_height_m: f64,
}

impl Default for Atmosphere {
    /// Earth's density and pressure, on a scale height chosen to put the ceiling somewhere
    /// a Phase 1 rocket can plausibly reach.
    fn default() -> Self {
        Self {
            height_m: 70_000.0,
            sea_level_density: 1.225,
            sea_level_pressure_pa: 101_325.0,
            scale_height_m: 5_600.0,
        }
    }
}

impl Atmosphere {
    pub fn density_at(&self, altitude_m: f64) -> f64 {
        if altitude_m >= self.height_m {
            return 0.0;
        }
        // Below sea level (in a valley, or clipped into terrain) the exponential would keep
        // growing. Clamping means the densest the air ever gets is sea level.
        self.sea_level_density * (-altitude_m.max(0.0) / self.scale_height_m).exp()
    }

    /// Ambient pressure, pascals. Engines interpolate their Isp against this.
    pub fn pressure_at(&self, altitude_m: f64) -> f64 {
        if altitude_m >= self.height_m {
            return 0.0;
        }
        self.sea_level_pressure_pa * (-altitude_m.max(0.0) / self.scale_height_m).exp()
    }

    /// Ambient pressure as a fraction of sea level, in `0.0..=1.0`.
    pub fn pressure_ratio_at(&self, altitude_m: f64) -> f64 {
        self.pressure_at(altitude_m) / self.sea_level_pressure_pa
    }
}

/// Aerodynamic properties of a part.
///
/// DESIGN.md: "Aerodynamic drag is computed per-part: cross-sectional area × drag
/// coefficient × dynamic pressure, not CFD."
#[derive(Component, Debug, Clone, Copy)]
pub struct DragSurface {
    /// Frontal cross-section, m².
    pub area_m2: f64,
    /// Drag coefficient, dimensionless.
    pub cd: f64,
}

/// Applies aerodynamic drag to every part with a [`DragSurface`].
///
/// `F = ½ · ρ · v² · Cd · A`, opposing the velocity vector.
///
/// # What this does not model
///
/// Occlusion. Every part presents its full frontal area, so a five-part stack has five
/// times the drag of its nose cone — the parts behind the first one are not shadowed by it.
/// KSP1 has the same problem and shipped with it for years. Fixing it properly means
/// deciding which parts are exposed along the velocity vector, which is a real piece of
/// work and is logged as tech debt rather than guessed at here.
///
/// Lift, angle of attack and the aerodynamic torque that makes a rocket weathervane are all
/// absent for the same reason: they need a model of where the force acts, not just how big
/// it is. Drag here acts at the centre of mass, so it slows a vessel without ever turning it.
///
/// # Ordering
///
/// `FixedUpdate`, in `PhysicsSchedule::Produce`, alongside thrust and gravity.
pub fn apply_drag(
    body: Res<CelestialBody>,
    atmosphere: Res<Atmosphere>,
    mut parts: Query<(&SimPosition, &SimVelocity, &DragSurface, &mut PendingForces)>,
) {
    for (position, velocity, surface, mut forces) in &mut parts {
        let speed = velocity.0.length();
        if speed <= 0.0 {
            continue;
        }

        let density = atmosphere.density_at(body.altitude_of(position.0));
        if density <= 0.0 {
            continue;
        }

        let magnitude = 0.5 * density * speed * speed * surface.cd * surface.area_m2;
        forces.add_force(-velocity.0 / speed * magnitude);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn density_falls_off_exponentially_and_stops_at_the_ceiling() {
        let atmosphere = Atmosphere::default();

        assert!((atmosphere.density_at(0.0) - 1.225).abs() < 1e-9);

        // One scale height up is exactly 1/e of sea level. This is the property the whole
        // model rests on, so it is worth pinning rather than assuming.
        let one_scale_height = atmosphere.density_at(atmosphere.scale_height_m);
        assert!((one_scale_height - 1.225 / std::f64::consts::E).abs() < 1e-9);

        assert_eq!(atmosphere.density_at(atmosphere.height_m), 0.0);
        assert_eq!(atmosphere.density_at(200_000.0), 0.0);
    }

    /// Terminal velocity is the number a player actually feels, so it is the one worth
    /// asserting: a 5 t stack must not be falling at supersonic speeds all the way down,
    /// and must not be floating either.
    #[test]
    fn terminal_velocity_is_in_a_sane_range() {
        let atmosphere = Atmosphere::default();
        let mass_kg = 5_000.0;
        let cd_a = 0.3 * 1.23 * 5.0; // five 1.25 m parts

        let terminal = (2.0 * mass_kg * 9.81 / (atmosphere.sea_level_density * cd_a)).sqrt();
        assert!(
            (100.0..400.0).contains(&terminal),
            "terminal velocity {terminal} m/s is outside anything plausible"
        );
    }

    #[test]
    fn pressure_ratio_is_one_at_sea_level_and_zero_in_space() {
        let atmosphere = Atmosphere::default();
        assert!((atmosphere.pressure_ratio_at(0.0) - 1.0).abs() < 1e-12);
        assert_eq!(atmosphere.pressure_ratio_at(100_000.0), 0.0);
    }
}
