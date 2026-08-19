//! Turning a [`PartShape`] into a collider and a mesh.
//!
//! Both come from the same declaration, so a part cannot look one size and collide at
//! another — which is what happened while colliders were derived from attach-node spacing
//! and meshes from a `.glb` that, for four of the five stock parts, did not exist.
//!
//! The meshes are compositions of Bevy primitives rather than generated vertex data. A
//! rocket engine really is a tube with a cone under it, and three primitives that read
//! correctly at a glance beat a hand-rolled mesh generator nobody will maintain.

use bevy::prelude::*;
use bevy_rapier3d::prelude::Collider;

use crate::sdk::shape::{PartShape, ShapeKind};

/// One piece of a part's placeholder mesh: a shape, and where to put it.
pub struct MeshPiece {
    pub mesh: Mesh,
    pub offset: Vec3,
    /// Darker pieces read as recesses — nozzle bells, the gap in a decoupler.
    pub shade: f32,
}

/// Builds the collider for a part.
///
/// `half_height` already accounts for the gap that keeps neighbouring parts' colliders from
/// touching (see `assembly::COLLIDER_GAP_M`), and `centre_offset` for parts whose origin is
/// not their geometric centre.
///
/// Every kind currently collides as a cylinder. The bell of an engine and the splay of a
/// landing leg are visual: a convex hull of the real silhouette would be more faithful and
/// would also mean a rocket resting on its nozzle rim rather than its base, which is a
/// physics change nobody has asked for. The one exception is the landing leg, whose whole
/// purpose is a wider footprint — it collides at its full splayed radius.
pub fn collider_for(shape: &PartShape, half_height: f64, centre_offset: f64) -> Collider {
    let radius = match shape.kind {
        ShapeKind::Decoupler => shape.radius_m * DECOUPLER_FLARE,
        ShapeKind::LandingLeg => shape.radius_m,
        _ => shape.radius_m,
    };

    Collider::compound(vec![(
        Vec3::new(0.0, centre_offset as f32, 0.0),
        Quat::IDENTITY,
        Collider::cylinder(half_height as f32, radius as f32),
    )])
}

/// How much wider than the stack a decoupler sits, so the seam is visible.
const DECOUPLER_FLARE: f64 = 1.08;

/// Fraction of an engine's length taken up by the nozzle bell.
const ENGINE_BELL_FRACTION: f32 = 0.55;

/// How far in the top of the nozzle bell is pinched, as a fraction of the part radius.
const ENGINE_THROAT_FRACTION: f32 = 0.45;

/// Builds the placeholder mesh for a part, as one or more primitives.
///
/// `height` is the part's full length and `centre_offset` where its middle sits relative to
/// its origin — both in metres, both already known to the collider.
pub fn mesh_for(shape: &PartShape, height: f64, centre_offset: f64) -> Vec<MeshPiece> {
    let radius = shape.radius_m as f32;
    let height = height as f32;
    let centre = centre_offset as f32;

    match shape.kind {
        ShapeKind::Cylinder => vec![MeshPiece {
            mesh: Cylinder::new(radius, height).into(),
            offset: Vec3::new(0.0, centre, 0.0),
            shade: 1.0,
        }],

        // A body with a flared nozzle underneath. The bell is a truncated cone, narrow where
        // it meets the body and wide at the exit, which is the silhouette that makes an
        // engine recognisable as an engine at a glance.
        ShapeKind::Engine => {
            let body_height = height * (1.0 - ENGINE_BELL_FRACTION);
            let bell_height = height * ENGINE_BELL_FRACTION;
            let bottom = centre - height / 2.0;
            vec![
                MeshPiece {
                    mesh: Cylinder::new(radius * 0.8, body_height).into(),
                    offset: Vec3::new(0.0, bottom + bell_height + body_height / 2.0, 0.0),
                    shade: 1.0,
                },
                MeshPiece {
                    mesh: ConicalFrustum {
                        radius_top: radius * ENGINE_THROAT_FRACTION,
                        radius_bottom: radius,
                        height: bell_height,
                    }
                    .into(),
                    offset: Vec3::new(0.0, bottom + bell_height / 2.0, 0.0),
                    shade: 0.55,
                },
            ]
        }

        // Deliberately wider than the stack and dark, so the place the rocket is going to
        // come apart is obvious before it does.
        ShapeKind::Decoupler => vec![MeshPiece {
            mesh: Cylinder::new(radius * DECOUPLER_FLARE as f32, height).into(),
            offset: Vec3::new(0.0, centre, 0.0),
            shade: 0.5,
        }],

        ShapeKind::NoseCone => vec![MeshPiece {
            mesh: Cone { radius, height }.into(),
            offset: Vec3::new(0.0, centre, 0.0),
            shade: 1.0,
        }],

        // A strut down to a footpad. The pad is what actually stops the rocket tipping over,
        // so it is drawn where the collider is.
        ShapeKind::LandingLeg => {
            let bottom = centre - height / 2.0;
            let pad_height = height * 0.18;
            vec![
                MeshPiece {
                    mesh: Cylinder::new(radius * 0.14, height).into(),
                    offset: Vec3::new(0.0, centre, 0.0),
                    shade: 0.8,
                },
                MeshPiece {
                    mesh: Cylinder::new(radius, pad_height).into(),
                    offset: Vec3::new(0.0, bottom + pad_height / 2.0, 0.0),
                    shade: 0.6,
                },
            ]
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every piece of every shape has to stay inside the part's declared extent, or a mesh
    /// pokes through its neighbour in the stack and the collider gap stops looking like a
    /// gap. Cheap to assert, and easy to break when adding a kind.
    #[test]
    fn mesh_pieces_stay_within_the_declared_extent() {
        let height = 2.0;
        let centre = 0.0;
        for kind in [
            ShapeKind::Cylinder,
            ShapeKind::Engine,
            ShapeKind::Decoupler,
            ShapeKind::NoseCone,
            ShapeKind::LandingLeg,
        ] {
            let shape = PartShape {
                kind,
                radius_m: 0.625,
            };
            for piece in mesh_for(&shape, height, centre) {
                let half = height as f32 / 2.0;
                assert!(
                    piece.offset.y.abs() <= half,
                    "{kind:?} places a piece at y={} outside +/-{half}",
                    piece.offset.y
                );
            }
        }
    }
}
