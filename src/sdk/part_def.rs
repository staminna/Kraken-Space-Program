//! What a Lua part definition compiles into.
//!
//! This is the answer to the EXECUTION.md question *"what struct does a Lua part
//! definition compile into at load time?"* — a plain Rust struct, loaded once at startup,
//! then turned into ECS components by `vessel::assembly`. Nothing here is a component:
//! these are the *blueprints*, not the parts. One `PartDefinition` produces many part
//! entities.
//!
//! # Units
//!
//! Lua part files are written in the units an engineer would use — tonnes, kilonewtons,
//! seconds. **Everything in this module is SI**: kilograms, newtons, metres. The
//! conversion happens exactly once, in `api::parts`, at load time.
//!
//! Mixing units is how you lose a Mars orbiter. The rule is: Lua is human-facing and may
//! use convenient units; the moment a value crosses into Rust it is SI and stays SI.

use bevy::math::DVec3;

/// A part blueprint, loaded from one `part { ... }` block in a `.lua` file.
#[derive(Debug, Clone)]
pub struct PartDefinition {
    /// Stable identity, e.g. `kraken.engine.spark`. Save files reference this string, so
    /// changing it breaks existing saves.
    pub id: String,
    // Shown by the editor's part browser (Phase 4). Parsed now because throwing away
    // authored content and re-adding the plumbing later is the more expensive order.
    #[allow(dead_code)]
    pub display_name: String,
    #[allow(dead_code)]
    pub manufacturer: String,
    #[allow(dead_code)]
    pub description: String,
    pub categories: Vec<String>,
    /// Dry mass in **kilograms** (Lua declares tonnes).
    pub dry_mass_kg: f64,
    /// Mesh path relative to the asset root, e.g. `parts/kraken_stock/fueltank_1.glb`.
    /// `None` means "no mesh authored yet" — assembly substitutes a primitive.
    pub geometry: Option<String>,
    pub attach_nodes: Vec<AttachNode>,
    pub modules: Vec<PartModuleDef>,
    /// Drag coefficient, dimensionless. Lua may declare `drag_coefficient`; the default is
    /// a blunt cylinder, which is what most parts are and what every part looks like until
    /// somebody models a nose cone.
    pub drag_coefficient: f64,
}

/// Default drag coefficient for a part that does not declare one.
///
/// 0.3 is a smooth cylinder in axial flow. It is deliberately not the 0.8–1.2 of a flat
/// disc: with no occlusion model every part in a stack presents its full frontal area, so a
/// per-part coefficient tuned as if each were alone in the airstream would give a five-part
/// rocket about five times too much drag.
pub const DEFAULT_DRAG_COEFFICIENT: f64 = 0.3;

impl PartDefinition {
    /// Finds an attach node by name (`"top"`, `"bottom"`, ...).
    pub fn node(&self, name: &str) -> Option<&AttachNode> {
        self.attach_nodes.iter().find(|node| node.name == name)
    }
}

/// A point where another part can connect.
#[derive(Debug, Clone)]
pub struct AttachNode {
    /// Key from the Lua `attach_nodes` table — `top`, `bottom`, and later `radial_1` etc.
    pub name: String,
    /// Position in part-local space, metres.
    pub position: DVec3,
    /// Node size class. 1 = 0.625 m, 2 = 1.25 m. Parts with mismatched sizes should not
    /// connect (not enforced yet).
    pub size: u32,
    /// Force along the node axis that breaks the joint, in **newtons** (Lua declares kN).
    pub tensile_strength_n: f64,
    /// Force across the node axis that breaks the joint, in **newtons**.
    pub shear_strength_n: f64,
}

/// A module attached to a part.
///
/// design.md: "Part modules are components, not objects." This enum exists only to carry
/// data from Lua to the assembly system, which inserts the matching *component*. There is
/// no `PartModule` trait, no virtual dispatch, and no lifecycle methods — adding a module
/// type means adding a variant here, a component in `part_modules/`, and a system.
#[derive(Debug, Clone)]
pub enum PartModuleDef {
    Engine(EngineDef),
    ResourceContainer(ResourceContainerDef),
    Decoupler(DecouplerDef),
}

#[derive(Debug, Clone)]
pub struct EngineDef {
    /// Vacuum thrust in **newtons** (Lua declares kN).
    pub thrust_n: f64,
    /// Vacuum specific impulse, seconds.
    pub isp_vac_s: f64,
    /// Sea-level specific impulse, seconds.
    pub isp_sl_s: f64,
    /// Propellant mixture as `(resource, mass fraction)`. Fractions should sum to 1.
    pub propellants: Vec<(String, f64)>,
}

#[derive(Debug, Clone)]
pub struct ResourceContainerDef {
    pub resource: String,
    /// Starting quantity, kilograms.
    pub amount_kg: f64,
    /// Capacity, kilograms.
    pub max_kg: f64,
}

#[derive(Debug, Clone)]
pub struct DecouplerDef {
    /// Which stage fires this decoupler. Lower numbers fire first.
    pub stage: u32,
    /// Attach node released when it fires.
    pub node: String,
    /// Separation impulse in **newtons** (Lua declares kN).
    pub ejection_force_n: f64,
}
