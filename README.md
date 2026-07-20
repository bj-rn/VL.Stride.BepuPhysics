# VL.Stride.BepuPhysics

[BepuPhysics v2](https://github.com/bepu/bepuphysics2) rigid body physics for [vvvv gamma](https://vvvv.org),
based on [Stride.BepuPhysics](https://github.com/stride3d/stride/tree/master/sources/engine/Stride.BepuPhysics)
and integrated with the VL.Stride entity system: connect a `Body` or `Static` to an Entity
exactly like a ModelComponent, and physics drives the entity's transform.

## Features

- `Body` (dynamic/kinematic) and `Static` components, category `Stride.Physics.Bepu.Components`
- `Character` component: a walking, jumping physics character (Move / TryJump / CharacterState)
- Collider shapes: Box, Sphere, Capsule, Cylinder, Triangle, Mesh (from any Model with CPU accessible mesh data), ConvexHull, Empty
- Runtime hull baking: HullFromModel and HullFromPoints produce the ConvexHullCollider's hull data (single hull, the convex envelope; multi hull decomposition only exists in the Stride editor's asset pipeline)
- Async hull baking: HullFromPoints (Async), HullsFromPointGroups (Async) and HullFromModel (Async) reduce point clouds to their convex hull on a background thread, incl. one hull per point group for per glyph text physics with VL.Stride.Text3d (see below)
- All 30 Bepu constraint types (BallSocket, Hinge, motors, servos, limits, Weld, Area, Volume, ...)
- Runtime constraint access: GetConstraints on a body, GetConstraintBodies, ConstraintInfo, spring / motor / servo settings operations, applied force readout, plus type specific settings operations for all 30 constraint types
- Queries: RayCast, RayCastPenetrating, SweepCast, SweepCastPenetrating (optionally with a rotating shape via the Angular Velocity pin) and Overlap, plus per collidable RayCast and RayCastPenetrating variants that test a single body or static
- Per-collidable contact events (started / touching / stopped)
- Trigger volumes: overlap detection without collision response (entered / exited)
- SimulationSettings: gravity, fixed timestep, solver iterations, collision matrix, all live
- Collision filtering: per collidable layer (32 layers, pair matrix) plus collision groups for fine grained rules like chain links ignoring their neighbours
- Per step hooks: the SimulationUpdate node reports every physics step (counts and observables) for frame rate independent forces
- Debug view: the ColliderShapes node builds translucent wireframed shape entities for every collider in the simulation, following the raw physics poses, rendered through the normal pipeline (works with the default SceneWindow). An opt in Instanced mode draws all shapes sharing a mesh in one draw call for very large simulations, fill only, wireframe lines are not supported there
- Diagnostics: the BepuInfo node reports configuration and simulation state, body/static counts and, with a component connected, its attachment chain and pose (both debug nodes in category `Stride.Physics.Bepu.Debug`)
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
4. Rebuild, run the compile checks and the patch verification (see Testing), bump the package version in the
   nuspec, add a row to this table and release.

## Quick start

Open `help/Reference Overview Bepu Physics.vl` for a tour of the package, or browse
the `Stride.Physics.Bepu` category in the node browser. Its subcategories: `Components`
(Body, Static, Character and their operations), `Colliders`, `Constraints`, `Events`
(ContactEvents, Trigger), `Simulation` (settings, reset, per step hooks and the scene
queries) and `Debug`. Every node has a Reference patch in `help/`, organized in the same
way, including one patch per constraint type.

Basics:
1. Create an Entity (any VL.Stride entity node, e.g. `Box` from `Stride.Models`).
2. Create a `Body` node, connect a `BoxCollider` to its `Colliders` pin group.
3. Connect the Body output to the entity's `Components` input.
4. The entity's transform at attach time is the initial pose; from then on physics owns it.
   Use `Teleport`/`SetTargetPose`/impulse operation nodes to move bodies.

Notes:
- A collider shape instance must only be used by **one** collidable (Body, Static or
  Character). Sharing is not rejected, but the shape only tracks one owner, so property
  changes and detachment go to the wrong body.
- A component instance can only be attached to **one** Entity.
- Simulation settings (gravity etc.) apply globally via the `SimulationSettings` node.
- **Do not wire a matrix into a physics-driven entity's `Transformation` pin**, the entity
  node re-applies it every frame and overrides the physics. Use the Body node's `TeleportTo`
  pin instead: it places the body only when the value changes.
- `Body.ResetPose` (bang) re-teleports a single body to its `TeleportTo` pose and zeroes
  its velocities (it does nothing while `TeleportTo` is unconnected); the
  `SimulationReset` node snapshots and
  restores many bodies at once (auto-captures the first frame with matching bodies,
  `Capture` for a new start state, `Reset` to restore). By default it captures **all**
  bodies; the optional Collision Mask and Collision Group Id pins narrow the capture to
  a layer set and/or group.
- The simulation starts ticking during vvvv startup, before the render window opens,
  use `SimulationSettings.Enabled` to stage a scene frozen, or bang `Reset` once visible.
- Fast bodies (long drops) can tunnel through thin geometry in `Discrete` mode, set the
  Body's `Continuous Detection` pin to `Continuous` for swept collision.

## Async hull baking

`HullFromPoints (Async)`, `HullsFromPointGroups (Async)` and `HullFromModel (Async)`
run the convex hull computation on a background thread (`In Progress` reports
activity; the last completed hull stays active meanwhile, and rapid input changes are
coalesced into one bake). The baked hull carries only the reduced hull vertices,
typically dozens instead of thousands of mesh vertices, so the hull build the engine
unavoidably runs on the main thread when the collider attaches becomes trivial.

`HullsFromPointGroups (Async)` bakes one hull per point group into a single collider,
made for per glyph text physics: the async mesh nodes of
[VL.Stride.Text3d](https://github.com/bj-rn/VL.Stride.Text3d) (2.4.0 or newer) output
matching `Point Groups`. See `help/Colliders/HowTo Async Hull Baking.vl`
(self contained) and the end to end `help/HowTo Physical 3d Text.vl` (needs
VL.Stride.Text3d installed, the package itself does not depend on it).

Notes:
- Degenerate input (fewer than 4 distinct points, or all points in one plane) outputs
  null instead of failing at attach; degenerate groups are skipped.
- Unlike the sync HullFromPoints, the async hulls carry triangle data, so the
  ColliderShapes debug node can draw them.
- MeshCollider has no async variant: the engine cooks the mesh on the main thread at
  attach, proportional to the triangle count, and that cannot be moved off thread.

What runs where:

| Work | Thread |
|---|---|
| Convex hull computation / point reduction | background (private per task BufferPool) |
| HullFromModel GPU readback | main (needs the graphics device) |
| Engine hull build at collider attach | main, unavoidable; trivial with reduced points |
| MeshCollider cook at attach | main, unavoidable; proportional to triangles |
| Collider (re)assign bookkeeping | main |

Measured attach cost (headless, Release, Ryzen 7 PRO 5850U; 42 extruded text glyphs,
21168 raw vertices reduced to 1424 hull points; hull/mesh builds as the engine runs
them at attach):

| Hull build at attach | ms |
|---|---|
| whole-text hull over all 21168 raw vertices (sync path) | 4.3 |
| whole-text hull over 46 reduced points (HullFromPoints (Async)) | 0.15 |
| 42 per-glyph hulls over raw point groups | 14.9 |
| 42 per-glyph hulls over reduced points (HullsFromPointGroups (Async)) | 5.8 |
| Bepu Mesh cook over 7056 triangles (MeshCollider proxy) | 7.5 |

So async baking minimizes but does not eliminate the attach cost: a single reduced
hull attaches for free; per glyph hulls keep a per-hull engine overhead (still well
under a frame, and paid once per completed bake, not per keystroke).

## Development

- `src/` builds with `dotnet build` into `lib/net8.0/`, vvvv loads the DLL from there,
  so restart vvvv to pick up changes. The DLL and its XML docs are committed
  (clone-and-go package repository), so rebuild before committing library changes.
- For debugging, create a local `.vscode/launch.json` (not committed) launching
  `vvvv.exe --package-repositories <parent of this repo> -o VL.Stride.BepuPhysics.vl`.
- `src/Internal/BackgroundComputation.cs` (the poll based helper behind the async hull
  nodes) is an intentional copy of `src/Core/BackgroundComputation.cs` in
  [VL.Stride.Text3d](https://github.com/bj-rn/VL.Stride.Text3d): only namespace and
  visibility differ. Duplicating 75 stable lines beats a cross package dependency (or
  a shared micro nuget) that would couple the two packages' releases. When changing
  the helper, change it in both repos; each carries the same semantics tests
  (`BackgroundComputationTests`), so a divergence shows up in whichever suite was not
  updated.

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

### 1. Headless compile checks (primary)

```
dotnet test tests\VL.Stride.BepuPhysics.Tests
```

`tests/` contains an NUnit project using **VL.TestFramework**: it boots the VL compiler
headlessly and verifies that the main `.vl` document and every help patch compile without
errors. Each document runs through the full compiler pipeline (load, dependency
resolution, type checking, code emission) but nothing is executed, no window opens and no
physics steps. Expect around 20 to 30 seconds for the whole suite. Run it after any C#
pin or signature change and after adding or generating patches, it is exactly the check
that catches a renamed pin breaking some help patch you did not think of. vvvv can stay
open while the tests run.

Useful variations:

```
dotnet test tests\VL.Stride.BepuPhysics.Tests --filter "Name~MainDocumentCompiles"   # just the main document
dotnet test tests\VL.Stride.BepuPhysics.Tests --filter "Name~HelpPatchCompiles"      # all help patches
dotnet test tests\VL.Stride.BepuPhysics.Tests --filter "Name~Weld"                   # a specific patch (path is the test name)
dotnet test tests\VL.Stride.BepuPhysics.Tests --logger "console;verbosity=detailed"  # full compiler messages on failure
```

Against a different vvvv installation (PowerShell):

```
$env:VVVV_DIR = "D:\vvvv\vvvv_gamma_7.4-win-x64"; dotnet test tests\VL.Stride.BepuPhysics.Tests
```

Preconditions: a local vvvv installation (the fixture's default path matches the
author's machine, on any other machine set the `VVVV_DIR` environment variable as shown
above) and the runtime packages (Stride.BepuPhysics, BepuPhysics, BepuUtilities)
installed in vvvv's user nugets directory
(`%LOCALAPPDATA%\vvvv\gamma\nugets`), which the test fixture adds to its search paths
automatically. The `help/HowTo Physical 3d Text.vl` compile check additionally needs
VL.Stride.Text3d (2.4.0 or newer) resolvable: a sibling checkout next to this repo
works out of the box, otherwise install the nuget. Because of these local requirements
the tests are a verification tool for this machine rather than a CI gate for now.

Important: `TestEnvironmentLoader.Load` is called with `preCompilePackages: true` and that
must stay. Without pre compilation the headless host never loads the runtime assemblies of
referenced packages, and computing imported pin defaults crashes with an
ArgumentNullException in `ImportedParameterPinDefinitionSymbol.GetDefaultValue`
(`Enum.ToObject(null, ...)`) as soon as any referenced assembly declares an enum parameter
default. Importing plain VL.Stride is enough to trigger it, this wrapper is not involved.
With pre compilation the packages are processed like in a regular vvvv startup and all
runtime types resolve.

### 2. Patch verification via vvvv (runtime fallback)

```
powershell -File tools\verify-patches.ps1 [-VvvvExe <path\to\vvvv.exe>] [-Seconds 60]
```

Launches a real vvvv instance for **every patch in `help/`**, watches its stdout for
`Exception` / compile errors for 60 seconds each, then kills it. Exit code 0 means all
patches loaded and ran clean. Much slower than the compile checks (a minute per patch),
but it exercises what they cannot: the actual runtime, node import, entity attachment,
the lazy Bepu bootstrap and the physics loop. Use it before releases and whenever a
change could break behavior rather than compilation. Note it cannot judge *visual*
correctness, open the help patches yourself to see bodies fall, the pendulum swing, etc.
The `BepuInfo` node (category `Stride.Physics.Bepu.Debug`) helps there: it reports
simulation state, body/static counts, and, with a component connected to its `Collidable`
pin, that component's attachment chain and pose.

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
- `ISimulationUpdate` (the interface behind the SimulationUpdate node's internal
  carrier component) is an entity component interface in 4.2.1
  (`IComponent<ISimulationUpdate, SimulationUpdateProcessor>` with `Entity`, `Simulation`,
  `SimulationSelector` properties). Registration happens through the entity component
  system, which is why the node attaches a carrier component to a scene entity. Recheck
  the interface shape on the new version.
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
- `BepuSimulation.ThreadCount` is dead code in 4.2.1 AND on master: the property is read
  once in the constructor, where it can only ever hold its initializer value -1, so the
  thread count is always the automatic pick (upstream bug, report candidate). The wrapper
  therefore exposes no ThreadCount pin; BepuInfo reads the real count from the private
  ThreadDispatcher via reflection. If upstream fixes the property, expose it and drop the
  reflection.

## License

MIT
