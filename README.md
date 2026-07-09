# VL.Stride.BepuPhysics

[BepuPhysics v2](https://github.com/bepu/bepuphysics2) rigid body physics for [vvvv gamma](https://vvvv.org),
based on [Stride.BepuPhysics](https://github.com/stride3d/stride/tree/master/sources/engine/Stride.BepuPhysics)
and integrated with the VL.Stride entity system: connect a `Body` or `Static` to an Entity
exactly like a ModelComponent, and physics drives the entity's transform.

## Features

- `Body` (dynamic/kinematic) and `Static` components, category `Stride.Physics.Bepu`
- Collider shapes: Box, Sphere, Capsule, Cylinder, Triangle, Mesh (from any Model), ConvexHull, Empty
- All 30 Bepu constraint types (BallSocket, Hinge, motors, servos, limits, Weld, Area, Volume, ...)
- Queries: RayCast, RayCastPenetrating, SweepCast, Overlap
- Per-collidable contact events (started / touching / stopped)
- SimulationSettings: gravity, fixed timestep, solver iterations, collision matrix, all live
- Transform interpolation enabled by default for smooth visuals at any frame rate

## Installation

```
nuget install VL.Stride.BepuPhysics
```

All runtime dependencies (Stride.BepuPhysics, BepuPhysics, BepuUtilities) resolve
automatically. The package rides on the Stride version bundled with vvvv — the declared
`Stride.BepuPhysics` version must match it (vvvv gamma 7.x ships Stride **4.2.1.2487**,
check the About dialog).

## Quick start

Open `help/HowTo Falling Bodies.vl` — a dynamic box falling onto a static ground,
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

## License

MIT
