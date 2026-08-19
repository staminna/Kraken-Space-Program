# Kraken Space Program — Checklist

> If you cut a corner, write it here **before you close the PR**. Not later. Now.
> Future contributors will not remember it was intentional and will waste time wondering if it's a bug.
> If you fix a debt item, remove it from the table and note what you did in the commit message.

---

## Known Tech Debt

| # | Description | Where | Cut in Phase | Should be fixed by Phase |
|---|-------------|-------|--------------|--------------------------|
| 2 | Attitude control is one global torque constant applied to the root part, not reaction wheels and gimballed engines. Control authority is therefore identical on every vessel regardless of what it is built from, and the constant is sized by hand against the one test stack — 3 kN·m gives it about 50°/s after a second of full deflection. Replace with an `RcsThruster` / `ReactionWheel` / gimbal part module. | `src/vessel/control.rs` (`ATTITUDE_TORQUE_NM`) | 1 | 2 |
| 6 | Propellant crossfeed is "same vessel, same `FuelGroup`", where fuel groups are assigned at assembly and change at decoupler boundaries. Real crossfeed flows through attach nodes with per-node rules. The simple version behaves correctly for stacks; it will be wrong for anything with radial tanks. | `src/part_modules/engine.rs`, `src/vessel/assembly.rs` | 1 | 2 |
| 7 | A decoupler's `node` field is parsed but ignored — staging assumes every decoupler releases downward, which is true of stack decouplers and false of radial ones. | `src/vessel/staging.rs` | 1 | 2 |
| 8 | The test vessel is a hardcoded five-part list in Rust. The parts themselves come from Lua, but the arrangement does not. Goes away when the editor exists. | `src/vessel/assembly.rs` (`TEST_STACK`) | 1 | 4 |
| 9 | `kraken.engine.reliant` was written by the engine side, not by content authors, purely so the test stack could lift itself — the stock Spark is a 20 kN upper-stage engine and a fuelled 1.25 m tank weighs about 24 kN on the pad. Its numbers are plausible, not designed. Someone doing balance should own it. | `src/assets/parts/kraken_stock/engine_reliant.lua` | 1 | 2 |
| 10 | Part meshes are placeholder cylinders for every part except the fuel tank, which is the only one with a `.glb`. The definitions name meshes that do not exist yet; the loader warns once per part and substitutes a primitive. | `src/vessel/assembly.rs` | 1 | 4 |
| 11 | No landing legs, so a vessel that touches down with any lateral drift topples and the nose hits the ground above the impact limit. A dead-vertical touchdown survives; a 5 m/s drift does not. Legs are a part module with a suspension joint. | — | 1 | 2 |
| 12 | `SAFE_IMPACT_SPEED_MS` is one global number for every part. A landing leg and a fuel tank should not share an impact tolerance — it belongs in the Lua part definition next to `tensile_strength`. | `src/vessel/damage.rs` | 1 | 2 |
| 13 | A destroyed vessel loses its joints and its controls but its parts stay in the world as debris. Nothing despawns them, so wreckage accumulates for the lifetime of the session. Deliberate for now — despawning parts needs the joint graph and `RootPart`/camera references cleaned up with it. | `src/vessel/damage.rs` | 1 | 2 |
| 14 | `vessel_inertia` is a scalar approximation (largest principal component plus `m·d²`), not the inertia tensor, and it is recomputed from scratch for every vessel every tick. Fine for a five-part stack; it is `O(parts × vessels)` and will need caching before it is not. | `src/vessel/control.rs` | 1 | 2 |
| 15 | Drag has no occlusion: every part presents its full frontal area, so a five-part stack has five times the drag of its nose cone. KSP1 shipped with the same problem for years. Fixing it means deciding which parts are shadowed along the velocity vector. The stock `drag_coefficient` of 0.3 is deliberately low to compensate. | `src/celestial/atmosphere.rs` | 1 | 3 |
| 16 | Drag acts at the centre of mass, so it slows a vessel but never turns one. No lift, no angle of attack, no weathervaning — a rocket falls just as happily sideways as nose-first. All three need a model of where the force acts, not just how big it is. | `src/celestial/atmosphere.rs` | 1 | 3 |
| 17 | One celestial body and one atmosphere, both hardcoded `Default` impls in Rust. DESIGN.md wants them from Lua with pressure and temperature curves; this is the flat exponential it says to build first. | `src/celestial/` | 1 | 2 |
| 18 | Structural failure does not distinguish tension from compression — a tube buckles long before it snaps, but only one limit is authored per node, so both are compared against `tensile_strength`. | `src/physics/joints.rs` | 1 | 2 |

---

## Resolved

Kept briefly so reviewers can see what changed and why, then deleted.

| Was | Resolution |
|-----|------------|
| Structural failure not implemented (debt #1) | `joints::detect_joint_failures` reads the solver's constraint impulses each tick, converts them to axial and shear forces, and compares against the limits from Lua. Breaking requires the limit to be exceeded for three consecutive ticks — a rigid contact at 50 Hz stops a vessel inside one step, which is 217 kN through a 67 kN joint for a 1.8 m/s touchdown, so a single-tick test made every landing fatal. |
| No aerodynamic drag (debt #4) | `celestial::atmosphere` — exponential density, per-part `DragSurface`, force into `PendingForces`. A ballistic re-entry now peaks at 291 m/s at 4 km and *slows* to 274 m/s as the air thickens, instead of accelerating all the way down. |
| Engines always used sea-level Isp (debt #3) | Now interpolated on ambient pressure, which the atmosphere model made possible. |
| No crash detection (debt #5) | `physics/impact.rs` turns Rapier contacts into impact speeds, `vessel/damage.rs` breaks a vessel apart above `SAFE_IMPACT_SPEED_MS`. Two things had to be got right that are not obvious: the speed comes from the tick *before* the contact, because Rapier has already cancelled the velocity by the time the event is readable; and contacts between two parts of the same vessel are ignored, or the rocket destroys itself the first time it turns. |
| `eprintln!` instead of `tracing` | Enabled the `bevy_log` feature and replaced every call with `info!`/`debug!`/`warn!`. Note the original entry proposed adding a raw `tracing` dependency — that was the wrong fix; `bevy_log` is what installs the subscriber that makes `RUST_LOG` work. |
| `log_ball_position` spamming stderr every frame | Deleted along with the ball. The HUD replaced it. |
| `LocalOrigin` never updated | Now driven by `physics::krakensbane`, which keeps it equal to `WorldOrigin`. |
| Nothing carried `SimPosition` + `SyncRender` | Every part does. `physics::readback` derives `SimPosition` from Rapier each tick, and each part's visual entity follows it. |
| Physics interpolation not implemented | Implemented in `render_sync`, interpolating between the last two ticks by `Time<Fixed>::overstep_fraction`. Note this was a *second* bug in the same area: physics was also not actually running at 50 Hz — see EXECUTION.md's decision log. |
