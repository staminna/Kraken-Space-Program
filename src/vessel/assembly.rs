//! Turning [`PartDefinition`]s into a live vessel.
//!
//! This is where the SDK's data becomes the game's entities. It is the only place that
//! knows both what a part definition looks like and what a part entity looks like — the
//! loader has never heard of ECS components, and the physics systems have never heard of
//! Lua.

use bevy::math::DVec3;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::celestial::atmosphere::DragSurface;
use crate::part_modules::decoupler::Decoupler;
use crate::part_modules::engine::Engine;
use crate::part_modules::resource_container::{ResourceContainer, ResourceSlot};
use crate::physics::forces::PendingForces;
use crate::physics::joints::{self, JointStrength};
use crate::physics::readback::PhysicsDriven;
use crate::rendering::camera::FlightCamera;
use crate::rendering::render_sync::{
    PreviousSimPosition, SimPosition, SimTransformBundle, SyncRender,
};
use crate::sdk::loader::PartRegistry;
use crate::sdk::part_def::{PartDefinition, PartModuleDef};
use crate::vessel::components::{
    ActiveVessel, AttachedBelow, ControlState, CurrentStage, FuelGroup, PartConnections, PartId,
    PartMass, PartVisual, RootPart, Vessel, VesselId,
};

/// Height above the pad the stack is spawned at, metres.
///
/// Just enough that the bottom collider does not start interpenetrating the pad. Any more
/// and the vessel is dropped onto the pad at launch, which is a jolt the joints have to
/// absorb before anything has even happened.
const PAD_CLEARANCE_M: f64 = 0.02;

/// How much shorter than its attach nodes each part's collider is, metres.
///
/// # Why there is a gap at all
///
/// Parts are stacked so that one's bottom node sits exactly on the next one's top node. If
/// the colliders reach all the way to those nodes, every neighbouring pair is in permanent
/// exact contact — and then the contact solver is pushing them apart at the same moment the
/// joint is holding them together. The two fight, and the fight injects energy: measured on
/// the launch pad, a stationary five-part stack bounced at up to 9.5 m/s for the first third
/// of a second and momentarily loaded its joints to nine times their breaking strength.
///
/// A 2 cm gap removes the contact entirely. The joint, not the collider, is what holds the
/// parts at the right distance, and the gap is between *colliders* — the meshes still meet,
/// because visuals are separate entities.
const COLLIDER_GAP_M: f64 = 0.02;

/// Rotational damping applied to every part, in Rapier's per-second units.
///
/// Not physical — there is nothing in vacuum to damp against. It stands in for the
/// rotational inertia losses a real vessel has and, more practically, makes the rocket
/// controllable by hand: without it a tap of `W` sets up a tumble that never stops.
/// Deliberately weak enough that a deliberate roll still takes a second or two to bleed off.
const ANGULAR_DAMPING: f32 = 0.5;

/// The hardcoded test stack, bottom to top.
///
/// design.md Phase 1: "Load a hardcoded vessel from a part tree definition". The *parts*
/// come from Lua; only the arrangement is hardcoded. The editor that replaces this list is
/// Phase 4.
const TEST_STACK: [&str; 5] = [
    "kraken.engine.reliant",
    "kraken.fueltank.small.03",
    "kraken.decoupler.small",
    "kraken.engine.spark",
    "kraken.fueltank.small.03",
];

/// Builds the test rocket and makes it the active vessel.
pub fn spawn_test_rocket(
    mut commands: Commands,
    registry: Res<PartRegistry>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    mut camera: Single<&mut FlightCamera>,
) {
    let definitions: Vec<&PartDefinition> = TEST_STACK
        .iter()
        .filter_map(|id| match registry.get(id) {
            Some(definition) => Some(definition),
            None => {
                error!("test stack references unknown part '{id}' — skipping");
                None
            }
        })
        .collect();

    if definitions.is_empty() {
        error!("no parts available, cannot assemble a vessel");
        return;
    }

    let vessel = commands
        .spawn((
            Vessel {
                name: "Test Stack".into(),
            },
            ActiveVessel,
            ControlState::default(),
            CurrentStage(0),
        ))
        .id();

    let mut previous: Option<(Entity, &PartDefinition, DVec3)> = None;
    let mut fuel_group = 0u32;
    let mut root: Option<(Entity, Entity)> = None;
    let mut spawned: Vec<Entity> = Vec::new();
    let mut neighbours: Vec<(Entity, Entity)> = Vec::new();

    for definition in definitions {
        // Stack each part so its bottom node touches the previous part's top node.
        let position = match &previous {
            None => DVec3::new(0.0, PAD_CLEARANCE_M - bottom_of(definition), 0.0),
            Some((_, below, below_position)) => {
                *below_position + top_of(below) - bottom_of_vec(definition)
            }
        };

        // A decoupler starts a new propellant network: it will separate here, so fuel must
        // not flow across it.
        if definition
            .modules
            .iter()
            .any(|module| matches!(module, PartModuleDef::Decoupler(_)))
        {
            fuel_group += 1;
        }

        let (part, visual) = spawn_part(
            &mut commands,
            definition,
            position,
            vessel,
            FuelGroup(fuel_group),
            &mut meshes,
            &mut materials,
            &asset_server,
        );

        if let Some((below, below_def, _)) = previous {
            joints::attach(
                &mut commands,
                part,
                below,
                bottom_of_vec(definition),
                top_of(below_def),
                JointStrength {
                    tensile_n: definition
                        .node("bottom")
                        .map_or(0.0, |node| node.tensile_strength_n),
                    shear_n: definition
                        .node("bottom")
                        .map_or(0.0, |node| node.shear_strength_n),
                },
            );

            // Connections are recorded locally and inserted after the loop. Inserting
            // them here would overwrite: commands are deferred, so the component cannot be
            // read back to append to it, and each part has a neighbour on both sides.
            neighbours.push((below, part));
            commands.entity(part).insert(AttachedBelow(below));
        }

        // The topmost part is the root: it is the furthest from the engines, so torque
        // applied there rotates the stack the way a control pod at the nose would.
        root = Some((part, visual));
        spawned.push(part);
        previous = Some((part, definition, position));
    }

    for part in &spawned {
        let connected: Vec<Entity> = neighbours
            .iter()
            .filter_map(|(a, b)| match (*a == *part, *b == *part) {
                (true, _) => Some(*b),
                (_, true) => Some(*a),
                _ => None,
            })
            .collect();
        commands.entity(*part).insert(PartConnections(connected));
    }

    let Some((root_part, root_visual)) = root else {
        return;
    };
    commands.entity(root_part).insert(RootPart);

    // The camera follows the root's *visual* entity, not the part: the visual is what
    // carries the interpolated render-space transform.
    camera.target = Some(root_visual);
    // Look at the middle of the stack rather than its nose.
    camera.target_offset = Vec3::new(0.0, -3.0, 0.0);
    camera.distance = 30.0;

    info!(
        "assembled vessel 'Test Stack' from {} parts",
        TEST_STACK.len()
    );
}

#[allow(clippy::too_many_arguments)]
fn spawn_part(
    commands: &mut Commands,
    definition: &PartDefinition,
    position: DVec3,
    vessel: Entity,
    fuel_group: FuelGroup,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    asset_server: &AssetServer,
) -> (Entity, Entity) {
    let half_height =
        ((top_of(definition).y - bottom_of(definition)) / 2.0 - COLLIDER_GAP_M).max(0.05);
    let radius = node_radius(definition);
    // The part's origin is not necessarily its geometric centre — the Spark's nodes run
    // from 0 to -0.5 — so the collider is offset to sit between the nodes rather than
    // straddling the origin.
    let centre_offset = (top_of(definition).y + bottom_of(definition)) / 2.0;

    let part = commands
        .spawn((
            PartId(definition.id.clone()),
            PartMass {
                dry_kg: definition.dry_mass_kg,
            },
            VesselId(vessel),
            fuel_group,
            PhysicsDriven,
            PendingForces::default(),
            Transform::from_translation(position.as_vec3()),
            RigidBody::Dynamic,
            Collider::compound(vec![(
                Vec3::new(0.0, centre_offset as f32, 0.0),
                Quat::IDENTITY,
                Collider::cylinder(half_height as f32, radius as f32),
            )]),
            AdditionalMassProperties::Mass(definition.dry_mass_kg as f32),
            ExternalForce::default(),
            Velocity::default(),
            Sleeping::disabled(),
            // Nested because a bundle tuple tops out at 15 elements.
            (
                // Decouplers write here. Without the component the ejection impulse was
                // silently dropped — `get_mut` on a missing component is `Err`, not a panic.
                ExternalImpulse::default(),
                // Angular only. A rocket coasting in vacuum must not lose translational
                // speed, but without *some* rotational damping every attitude correction
                // leaves the stack tumbling forever, because nothing else removes angular
                // velocity. See `control::apply_attitude_control`.
                Damping {
                    linear_damping: 0.0,
                    angular_damping: ANGULAR_DAMPING,
                },
                // Swept-shape collision. At 50 Hz a body descending at 500 m/s moves 10 m
                // per tick and would step straight through the 10 m pad without ever
                // generating a contact. CCD costs only on bodies that are actually moving
                // fast, which is exactly when it is needed.
                Ccd::enabled(),
                // Impact damage listens for these — see `physics::impact`.
                ActiveEvents::COLLISION_EVENTS,
                // Frontal area from the part's own radius; the coefficient comes from Lua.
                DragSurface {
                    area_m2: std::f64::consts::PI * radius * radius,
                    cd: definition.drag_coefficient,
                },
                // Rapier only writes mass properties back into a component that already
                // exists. SAS needs the inertia to size its correction — see
                // `control::SAS_SETTLE_SECS`.
                ReadMassProperties::default(),
            ),
            SimTransformBundle {
                sim_position: SimPosition(position),
                previous_position: PreviousSimPosition(position),
                ..default()
            },
        ))
        .id();

    insert_modules(commands, part, definition);

    let visual = spawn_visual(
        commands,
        definition,
        part,
        position,
        half_height,
        radius,
        centre_offset,
        meshes,
        materials,
        asset_server,
    );
    commands.entity(part).insert(PartVisual(visual));

    (part, visual)
}

/// Inserts the components for a part's declared modules.
///
/// design.md: an engine part gets an `Engine` component, a tank gets a
/// `ResourceContainer`. No base class, no dispatch — just a match.
fn insert_modules(commands: &mut Commands, part: Entity, definition: &PartDefinition) {
    // A part may declare several `resource_container` blocks (the stock tank declares one
    // per propellant), so they are gathered into a single component. Inserting per module
    // would leave the tank holding only whichever resource happened to be declared last.
    let mut container = ResourceContainer::default();

    for module in &definition.modules {
        match module {
            PartModuleDef::Engine(engine) => {
                commands.entity(part).insert(Engine {
                    thrust_n: engine.thrust_n,
                    isp_vac_s: engine.isp_vac_s,
                    isp_sl_s: engine.isp_sl_s,
                    propellants: engine.propellants.clone(),
                    current_thrust_n: 0.0,
                });
            }
            PartModuleDef::ResourceContainer(slot) => {
                container.slots.push(ResourceSlot {
                    resource: slot.resource.clone(),
                    amount_kg: slot.amount_kg,
                    max_kg: slot.max_kg,
                });
            }
            PartModuleDef::Decoupler(decoupler) => {
                commands.entity(part).insert(Decoupler {
                    stage: decoupler.stage,
                    node: decoupler.node.clone(),
                    ejection_force_n: decoupler.ejection_force_n,
                    fired: false,
                });
            }
        }
    }

    if !container.slots.is_empty() {
        commands.entity(part).insert(container);
    }
}

/// Spawns the visual twin of a part.
///
/// Separate entity, by necessity — see `render_sync::SyncRender`.
#[allow(clippy::too_many_arguments)]
fn spawn_visual(
    commands: &mut Commands,
    definition: &PartDefinition,
    part: Entity,
    position: DVec3,
    half_height: f64,
    radius: f64,
    centre_offset: f64,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    asset_server: &AssetServer,
) -> Entity {
    let visual = commands
        .spawn((
            Transform::from_translation(position.as_vec3()),
            Visibility::default(),
            SyncRender { source: part },
        ))
        .id();

    // The mesh hangs off a child so the primitive fallback can carry the same centre
    // offset the collider uses without the parent transform — which render_sync owns —
    // having to know about it.
    let offset = Transform::from_xyz(0.0, centre_offset as f32, 0.0);

    match glb_path(definition) {
        Some(path) => {
            // The mesh and material are loaded directly rather than through
            // `SceneRoot(GltfAssetLabel::Scene(0))`. Spawning a glTF *scene* replays the
            // file's whole node graph through Bevy's reflection-based scene spawner, which
            // panics if the file carries any component type this build has not registered
            // — a real failure with a reduced feature set, and an odd thing to risk when a
            // part is a single mesh with a single material anyway.
            commands.spawn((
                Mesh3d(
                    asset_server.load(
                        GltfAssetLabel::Primitive {
                            mesh: 0,
                            primitive: 0,
                        }
                        .from_asset(path.clone()),
                    ),
                ),
                MeshMaterial3d(
                    asset_server.load::<StandardMaterial>(
                        GltfAssetLabel::Material {
                            index: 0,
                            is_scale_inverted: false,
                        }
                        .from_asset(path),
                    ),
                ),
                offset,
                ChildOf(visual),
            ));
        }
        None => {
            commands.spawn((
                Mesh3d(meshes.add(Cylinder::new(radius as f32, (half_height * 2.0) as f32))),
                MeshMaterial3d(materials.add(placeholder_material(definition))),
                offset,
                ChildOf(visual),
            ));
        }
    }

    visual
}

/// Returns the part's mesh path if the file actually exists.
///
/// Definitions name meshes that have not been modelled yet — `engine_spark.glb` is
/// declared and absent. Checking up front turns that into one warning at load and a
/// primitive placeholder, rather than an asset error every frame and an invisible part.
fn glb_path(definition: &PartDefinition) -> Option<String> {
    let path = definition.geometry.as_ref()?;
    // Must match AssetPlugin::file_path.
    if std::path::Path::new("src/assets").join(path).exists() {
        Some(path.clone())
    } else {
        warn!(
            "part '{}' declares geometry '{path}' which does not exist — using a placeholder",
            definition.id
        );
        None
    }
}

fn placeholder_material(definition: &PartDefinition) -> StandardMaterial {
    // Engines dark, tanks pale, everything else grey — enough to tell the stack apart at a
    // glance while the real meshes do not exist.
    let has = |name: &str| definition.categories.iter().any(|c| c == name);
    let base_color = if has("engine") {
        Color::srgb(0.25, 0.24, 0.26)
    } else if has("fuel tank") {
        Color::srgb(0.82, 0.82, 0.85)
    } else {
        Color::srgb(0.45, 0.45, 0.48)
    };

    StandardMaterial {
        base_color,
        perceptual_roughness: 0.55,
        metallic: 0.4,
        ..default()
    }
}

/// Y of the part's top attach node, as an offset vector.
fn top_of(definition: &PartDefinition) -> DVec3 {
    definition
        .node("top")
        .map_or(DVec3::ZERO, |node| node.position)
}

fn bottom_of_vec(definition: &PartDefinition) -> DVec3 {
    definition
        .node("bottom")
        .map_or(DVec3::ZERO, |node| node.position)
}

fn bottom_of(definition: &PartDefinition) -> f64 {
    bottom_of_vec(definition).y
}

/// Collider radius from the attach node size class: 1 = 0.625 m, 2 = 1.25 m diameter.
fn node_radius(definition: &PartDefinition) -> f64 {
    let size = definition
        .attach_nodes
        .iter()
        .map(|node| node.size)
        .max()
        .unwrap_or(1);
    match size {
        1 => 0.3125,
        2 => 0.625,
        other => 0.625 * f64::from(other) / 2.0,
    }
}
