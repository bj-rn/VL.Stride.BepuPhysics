using System.Text;
using Stride.Engine;
using Stride.Games;
using VL.Core;
using VL.Core.Import;
using VL.Lib.Basics.Resources;
using SBepu = global::Stride.BepuPhysics;
using SColliders = global::Stride.BepuPhysics.Definitions.Colliders;

namespace VL.Stride.BepuPhysics.Debug;

/// <summary>
/// Diagnostic: reports the state of the Bepu integration (configuration, game system, simulation, body counts).
/// Optionally connect a Body/Static component to diagnose its attachment chain.
/// </summary>
[ProcessNode]
public class BepuInfo : IDisposable
{
    private readonly IResourceHandle<Game> _gameHandle = AppHost.Current.Services.GetGameHandle();
    private readonly StringBuilder _sb = new();

    /// <param name="info">Human-readable state of the physics integration and, if connected, the component.</param>
    /// <param name="collidable">Optional: a Body or Static component to diagnose (attachment, collider, pose, sleep state).</param>
    public void Update(out string info, SBepu.CollidableComponent? collidable = null)
    {
        _sb.Clear();
        AppendGlobalInfo(_sb);
        if (collidable is not null)
        {
            _sb.Append(" || Component: ");
            AppendComponentInfo(_sb, collidable);
        }
        info = _sb.ToString();
    }

    private void AppendGlobalInfo(StringBuilder sb)
    {
        var game = _gameHandle.Resource;

        // Passive observation — GetService, not GetOrCreate, so this node doesn't hide bootstrap problems.
        var config = game.Services.GetService<SBepu.BepuConfiguration>();
        if (config is null)
        {
            sb.Append("BepuConfiguration: NOT created (no collidable has triggered the lazy bootstrap yet)");
            return;
        }

        var hasGameSystem = false;
        foreach (var system in game.GameSystems)
        {
            if (system.GetType().Name == "PhysicsGameSystem")
            {
                hasGameSystem = true;
                break;
            }
        }

        var sim = config.BepuSimulations.Count > 0 ? config.BepuSimulations[0] : null;
        if (sim is null)
        {
            sb.Append($"Config OK, PhysicsGameSystem: {hasGameSystem}, but no simulations!");
            return;
        }

        var bepu = sim.Simulation;
        sb.Append($"Config OK | PhysicsGameSystem: {hasGameSystem} | Sim Enabled: {sim.Enabled}, TimeScale: {sim.TimeScale} " +
                  $"| Gravity: {sim.PoseGravity} | Active bodies: {bepu.Bodies.ActiveSet.Count} " +
                  $"| Statics: {bepu.Statics.Count} | Threads: {GetThreadCount(sim)}");
    }


    // The engine's ThreadCount property is dead code (read once in the constructor where it
    // is always the default -1, upstream bug), the real count lives in the private readonly
    // ThreadDispatcher: always the automatic pick, ProcessorCount - 2 on machines with more
    // than 4 cores, otherwise ProcessorCount - 1, minimum 1.
    private static readonly System.Reflection.FieldInfo? ThreadDispatcherField =
        typeof(SBepu.BepuSimulation).GetField("_threadDispatcher",
            System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);

    private static int GetThreadCount(SBepu.BepuSimulation sim)
        => ThreadDispatcherField?.GetValue(sim) is global::BepuUtilities.IThreadDispatcher dispatcher
            ? dispatcher.ThreadCount
            : -1;

    private static void AppendComponentInfo(StringBuilder sb, SBepu.CollidableComponent collidable)
    {
        sb.Append(collidable.GetType().Name);
        sb.Append($" | Entity: {(collidable.Entity is null ? "NULL" : collidable.Entity.Name)}");
        sb.Append($" | InScene: {collidable.Entity?.Scene is not null}");
        sb.Append($" | AttachedToSim: {collidable.Simulation is not null}");

        var collider = collidable.Collider;
        sb.Append($" | Collider: {collider?.GetType().Name ?? "NULL"}");
        if (collider is SColliders.CompoundCollider compound)
            sb.Append($" (children: {compound.Colliders.Count})");

        if (collidable is SBepu.BodyComponent body && body.Simulation is not null)
            sb.Append($" | Pos: {body.Position} | Awake: {body.Awake}");
    }

    public void Dispose() => _gameHandle.Dispose();
}
