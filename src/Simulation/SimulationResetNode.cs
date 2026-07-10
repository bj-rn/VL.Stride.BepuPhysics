using Stride.Core.Mathematics;
using VL.Core.Import;
using SBepu = global::Stride.BepuPhysics;

namespace VL.Stride.BepuPhysics.Simulation;

/// <summary>
/// Captures the pose and velocity of every body in the simulation and restores them on demand —
/// a whole-simulation reset without wiring individual bodies.
/// By default the first frame with bodies present is captured automatically; bang Capture to
/// take a new snapshot (e.g. after spawning more bodies or arranging a new start state).
/// </summary>
[ProcessNode(Name = "SimulationReset")]
public class SimulationResetNode
{
    private readonly record struct BodySnapshot(
        SBepu.BodyComponent Component,
        Vector3 Position,
        Quaternion Orientation,
        Vector3 LinearVelocity,
        Vector3 AngularVelocity);

    private readonly List<BodySnapshot> _snapshot = new();
    private bool _hasCaptured;
    private bool _lastCapture;
    private bool _lastReset;

    /// <param name="capturedBodies">Number of bodies in the current snapshot.</param>
    /// <param name="simulation">The simulation to snapshot, from a SimulationSettings or GetSimulation node.</param>
    /// <param name="capture">Takes a new snapshot of all body poses and velocities. Connect a Bang.</param>
    /// <param name="reset">Restores all captured bodies to their snapshot state. Connect a Bang.</param>
    /// <param name="autoCapture">Automatically captures the first frame in which any bodies exist.</param>
    public void Update(
        out int capturedBodies,
        SBepu.BepuSimulation? simulation = null,
        bool capture = false,
        bool reset = false,
        bool autoCapture = true)
    {
        var captureEdge = capture && !_lastCapture;
        var resetEdge = reset && !_lastReset;
        _lastCapture = capture;
        _lastReset = reset;

        if (simulation is null)
        {
            capturedBodies = _snapshot.Count;
            return;
        }

        if (captureEdge || (autoCapture && !_hasCaptured && HasBodies(simulation)))
            Capture(simulation);

        if (resetEdge)
            Restore();

        capturedBodies = _snapshot.Count;
    }

    private static bool HasBodies(SBepu.BepuSimulation simulation)
    {
        var bodies = simulation.Simulation.Bodies;
        for (int setIndex = 0; setIndex < bodies.Sets.Length; setIndex++)
        {
            ref var set = ref bodies.Sets[setIndex];
            if (set.Allocated && set.Count > 0)
                return true;
        }
        return false;
    }

    private void Capture(SBepu.BepuSimulation simulation)
    {
        _snapshot.Clear();
        var bodies = simulation.Simulation.Bodies;
        // Set 0 is the active set; the others hold sleeping bodies.
        for (int setIndex = 0; setIndex < bodies.Sets.Length; setIndex++)
        {
            ref var set = ref bodies.Sets[setIndex];
            if (!set.Allocated)
                continue;
            for (int i = 0; i < set.Count; i++)
            {
                var component = simulation.GetComponent(set.IndexToHandle[i]);
                if (component is null)
                    continue;
                _snapshot.Add(new BodySnapshot(
                    component,
                    component.Position,
                    component.Orientation,
                    component.LinearVelocity,
                    component.AngularVelocity));
            }
        }
        _hasCaptured = true;
    }

    private void Restore()
    {
        foreach (var entry in _snapshot)
        {
            var component = entry.Component;
            // Skip bodies that have since been removed from the simulation.
            if (component.Simulation is null)
                continue;
            // Wake FIRST: mutating a sleeping body writes into the sleeping-set memory and
            // its broad-phase state, leaving the body intermittently frozen after restore.
            component.Awake = true;
            component.Teleport(entry.Position, entry.Orientation);
            component.LinearVelocity = entry.LinearVelocity;
            component.AngularVelocity = entry.AngularVelocity;
        }
    }
}
