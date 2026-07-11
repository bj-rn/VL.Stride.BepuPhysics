# VL.Stride.BepuPhysics

[BepuPhysics v2](https://github.com/bepu/bepuphysics2) rigid body physics for [vvvv gamma](https://vvvv.org),
based on [Stride.BepuPhysics](https://github.com/stride3d/stride/tree/master/sources/engine/Stride.BepuPhysics)
and integrated with the VL.Stride entity system: connect a `Body` or `Static` to an Entity
exactly like a ModelComponent, and physics drives the entity's transform.

## Features

- `Body` (dynamic/kinematic) and `Static` components, category `Stride.Physics.Bepu`
- `Character` component: a walking, jumping physics character (Move / TryJump / CharacterState)
- Collider shapes: Box, Sphere, Capsule, Cylinder, Triangle, Mesh (from any Model), ConvexHull, Empty
- Runtime hull baking: HullFromModel and HullFromPoints produce the ConvexHullCollider's hull data (single hull, the convex envelope; multi hull decomposition only exists in the Stride editor's asset pipeline)
- All 30 Bepu constraint types (BallSocket, Hinge, motors, servos, limits, Weld, Area, Volume, ...)
- Runtime constraint access: GetConstraints on a body, ConstraintInfo, spring / motor / servo settings operations, applied force readout, plus type specific settings operations for all 30 constraint types
- Queries: RayCast, RayCastPenetrating, SweepCast, SweepCastPenetrating, Overlap
- Per-collidable contact events (started / touching / stopped)
- Trigger volumes: overlap detection without collision response (entered / exited)
- SimulationSettings: gravity, fixed timestep, solver iterations, collision matrix, all live
- Collision filtering: per collidable layer (32 layers, pair matrix) plus collision groups for fine grained rules like chain links ignoring their neighbours
- Per step hooks: the SimulationUpdate node reports every physics step (counts and observables) for frame rate independent forces
- Transform interpolation enabled by default for smooth visuals at any frame rate

## Installation

```
nuget install VL.Stride.BepuPhysics -prerelease
```

All runtime dependencies (Stride.BepuPhysics, BepuPhysics, BepuUtilities) resolve
automatically. The `-prerelease` flag is required because BepuPhysics only exists as a
prerelease package on nuget.org.

## Compatibility

The package rides on the Stride version bundled with vvvv, the pinned
`Stride.BepuPhysics` version must match it exactly (check the About dialog in vvvv).

| vvvv gamma | Stride     | VL.Stride.BepuPhysics |
|------------|------------|-----------------------|
| 7.x        | 4.2.1.2487 | 0.2.0 and up          |

When a new vvvv release updates the bundled Stride version, this package needs a
matching release:

1. Check the Stride version of the new vvvv (About dialog).
2. Bump `StrideVersion` (and `VLVersion` if vvvv changed too) in `Directory.Packages.props`.
3. Update the exact pins in `deployment/VL.Stride.BepuPhysics.nuspec`: the
   `Stride.BepuPhysics` version and the `BepuPhysics` beta it depends on
   (see the dependency list of the Stride.BepuPhysics package on nuget.org).
4. Rebuild, run the patch verification (see Testing), bump the package version in the
   nuspec, add a row to this table and release.

## Quick start

Open `help/HowTo Falling Bodies.vl`, a dynamic box falling onto a static ground,
or browse the `Stride.Physics.Bepu` category in the node browser.

Basics:
1. Create an Entity (any VL.Stride entity node, e.g. `Box` from `Stride.Models`).
2. Create a `Body` node, connect a `BoxCollider` to its `Colliders` pin group.
3. Connect the Body output to the entity's `Components` input.
4. The entity's transform at attach time is the initial pose; from then on physics owns it.
   Use `Teleport`/`SetTargetPose`/impulse operation nodes to move bodies.

Notes:
- A collider shape instance can only be used by **one** Body/Static.
- A component instance can only be attached to **one** Entity.
- Simulation settings (gravity etc.) apply globally via the `SimulationSettings` node.
- **Do not wire a matrix into a physics-driven entity's `Transformation` pin**, the entity
  node re-applies it every frame and overrides the physics. Use the Body node's `TeleportTo`
  pin instead: it places the body only when the value changes.
- `Body.ResetPose` (bang) re-drops a single body; the `SimulationReset` node snapshots and
  restores **all** bodies at once (auto-captures the first frame, `Capture` for a new
  start state, `Reset` to restore).
- The simulation starts ticking during vvvv startup, before the render window opens,
  use `SimulationSettings.Enabled` to stage a scene frozen, or bang `Reset` once visible.
- Fast bodies (long drops) can tunnel through thin geometry in `Discrete` mode, set the
  Body's `Continuous Detection` pin to `Continuous` for swept collision.

## Development

- `src/` builds with `dotnet build` into `lib/net8.0/`, vvvv loads the DLL from there,
  so restart vvvv to pick up changes. The DLL and its XML docs are committed
  (clone-and-go package repository), so rebuild before committing library changes.
- For debugging, create a local `.vscode/launch.json` (not committed) launching
  `vvvv.exe --package-repositories <parent of this repo> -o VL.Stride.BepuPhysics.vl`.

### Tools

- `tools/generate-constraint-typeops.py` regenerates
  `src/Constraints/ConstraintTypeOperations.cs`, the 60 type specific constraint get and
  set operations, from the constraint node definitions (pin types, defaults and doc texts
  are parsed from the node files). Run it after changing constraint node pins, then
  rebuild. The generated file is committed and marked do not edit by hand.
- `tools/verify-patches.ps1` launches a real vvvv instance for every help patch and
  watches for errors, see Testing below.

## Testing

There are two verification layers:

### 1. Patch verification via vvvv (primary)

```
powershell -File tools\verify-patches.ps1 [-VvvvExe <path\to\vvvv.exe>] [-Seconds 60]
```

Launches a real vvvv instance for **every patch in `help/`**, watches its stdout for
`Exception` / compile errors for 60 seconds each, then kills it. Exit code 0 means all
patches loaded and ran clean. This is the authoritative check, since it exercises the
actual runtime: node import, entity attachment, the lazy Bepu bootstrap and the physics
loop. Note it cannot judge *visual* correctness, open the help patches yourself to see
bodies fall, the pendulum swing, etc. The `BepuInfo` node (category `Stride.Physics.Bepu`)
helps there: it reports simulation state, body/static counts, and, with a component
connected to its `Collidable` pin, that component's attachment chain and pose.

### 2. Headless compile checks (currently disabled)

```
dotnet test
```

`tests/` contains an NUnit project using **VL.TestFramework**: it boots the VL compiler
headlessly (entry assembly = `vvvv.exe`, override the install location with the `VVVV_DIR`
environment variable) and verifies that the main `.vl` document and every help patch
compile without errors. The fixture is currently marked `[Explicit]` because the headless
host crashes while importing nodes whose *referenced assemblies* declare enum parameter
defaults (`ImportedParameterPinDefinitionSymbol.GetDefaultValue` →
`Enum.ToObject(null, ...)`, it cannot resolve the enum's runtime type). The same
documents compile and run fine in real vvvv; the issue is a candidate for an upstream
report to vvvv. Once fixed, remove the `[Explicit]` attribute and the checks become
CI-ready (`dotnet test` in the GitHub workflow).

## Upgrading to a newer Stride.BepuPhysics

This section collects what to check and change in this wrapper when moving past the
pinned `Stride.BepuPhysics` 4.2.1.2487. The release steps themselves (version pins,
nuspec, rebuild, verification) are in the Compatibility section above. The deltas below
were collected by comparing the pinned assembly against the upstream master branch
(stride3d/stride, July 2026). Upstream moves on, so recheck each item at upgrade time.

### How to inspect an assembly version

Reflection is the authoritative source, XML docs only show documented members:

1. Create a console project referencing the new `Stride.BepuPhysics` version
   (copy this repo's `NuGet.config` next to it for the package feeds).
2. Load a type, walk `BaseType` up to `EntityComponent` and print properties and
   methods via `GetProperties`/`GetMethods` with `BindingFlags.DeclaredOnly`.
3. Alternatively diff the packages' XML doc files:
   `~/.nuget/packages/stride.bepuphysics/<version>/lib/net8.0/Stride.BepuPhysics.xml`.

### Known API deltas: 4.2.1.2487 vs upstream master

#### CharacterComponent was reworked

The pinned version is the old, velocity driven character:

| pinned 4.2.1.2487 | upstream master |
|---|---|
| `CharacterComponent : BodyComponent` | split into `CharacterComponentAbstract : BodyComponent` and `CharacterComponent` on top |
| `Move(Vector3 direction)` sets velocity from direction times `Speed` (direction length scales speed) | removed, set `MoveVector` (a `Vector2` in the body's local space, X = sideways, Y = forward) instead |
| `Velocity` property (world space) | still exists but `[Obsolete]`, forwards to `MoveVector` |
| `IsGrounded`, `IsJumping`, `Contacts` | `IsGrounded` kept (on the abstract base) |
| `JumpForce` | same name (`DataAlias("JumpSpeed")` hints at an even older name, irrelevant for us) |
| fixed internal ground test | new tunables: `MaximumHorizontalForce`, `MaximumVerticalForce`, `SlopeAngle`, `MinimumSupportDepth`, `MinimumSupportContinuationDepth`, `AirControlScale`, `AirControlForceScale`, `LocalUp` |

Wrapper impact when upgrading:
1. The `Move` operation must switch from calling `Move(Vector3)` to writing `MoveVector`,
   including the world space to local space conversion if the wrapper keeps its
   world space movement pin.
2. The Character node can expose the new tunables as pins (slope angle, air control,
   forces, local up).
3. Check whether `IsJumping` and `Contacts` still exist on master at upgrade time.

#### OverlapInfo was removed

The pinned `BepuSimulation.Overlap(shape, pose, ICollection<OverlapInfo>, mask)` fills
`OverlapInfo` records carrying `Collidable`, `PenetrationDirection` and
`PenetrationLength`. On master the overlap handlers only collect plain
`CollidableComponent`, the `OverlapInfo` type is gone.

Wrapper impact when upgrading:
1. The `Overlap` node's Output (`Spread<OverlapInfo>`) and the `Split (OverlapInfo)`
   operation lose their data source. Either drop the penetration outputs or check
   whether master offers a replacement API (`OverlapInfoStack` existed in 4.2.1 as a
   low level variant, check what survived).
2. Remove the `OverlapInfo` forward from `VL.Stride.BepuPhysics.vl`.

#### Unchanged (verified against master, recheck anyway)

- Query methods on `BepuSimulation`: `RayCast`, `RayCastPenetrating`, `SweepCast`,
  `SweepCastPenetrating` have the same shapes and semantics (T values in units of the
  direction's length, the wrapper normalizes).
- `Trigger`/`TriggerDelegate` exist in both, the wrapper no longer uses them (own
  handler with `NoContactResponse => true`).
- The 30 constraint components, collider types, `HitInfo`, `CollisionMask`,
  `InterpolationMode` and the contact event interfaces are structurally the same.
- `BepuSimulation` soft start behavior (`SolverSubStep` boosted by
  `SoftStartSubstepFactor` during the window) is the same on master, the
  SimulationSettings solver pin handling stays valid.

### Notes for the wrapper's own code at upgrade time

- `BepuSettingsBootstrap` mirrors what `BepuConfiguration.NewInstance` does lazily.
  If upstream changes its lazy bootstrap or the warning behavior, adjust or drop the
  bootstrap.
- `ISimulationUpdate` (relevant for the planned per step hook node) is an entity
  component interface in 4.2.1 (`IComponent<ISimulationUpdate, SimulationUpdateProcessor>`
  with `Entity`, `Simulation`, `SimulationSelector` properties). Registration happens
  through the entity component system, so a hook node needs a carrier component
  attached to an entity. Recheck the interface shape on the new version.
- `HullFromModel` replicates the engine's internal `ShapeCacheSystem.ExtractMeshBuffers`
  (the system is not public) using the public `AsReadable`/`Copy` mesh helpers from
  `Stride.Graphics`. On master the extraction additionally applies skeleton node
  transforms and offsets indices per mesh, recheck and mirror at upgrade time.
- Runtime VHACD decomposition stays unavailable: the only entry point in 4.2.1 is the
  editor's `HullAssetCompiler` (an `AssetCommand`, not callable at runtime). Recheck
  whether a newer version exposes a runtime API before promising multi hull baking.
- `CollisionGroup` indices are `ushort` in 4.2.1 but `short` on master. The
  CollisionGroup create operation clamps its int inputs, adjust the clamp range and the
  pin docs at upgrade time.

## License

MIT
