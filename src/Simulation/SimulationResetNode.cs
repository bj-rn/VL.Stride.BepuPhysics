using Stride.Core.Mathematics;
using VL.Core.Import;
using VL.Model;
using SBepu = global::Stride.BepuPhysics;

namespace VL.Stride.BepuPhysics.Simulation;

/// <summary>
/// Captures the pose and velocity of bodies in the simulation and restores them on demand,
/// a simulation reset without wiring individual bodies. Which bodies are captured can be
/// narrowed with the Collision Mask and Collision Group Id filters; by default all bodies
/// are captured. Use several SimulationReset nodes with different filters to reset different
/// sets of bodies independently, each node keeps its own snapshot.
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

    /// <param name="capturedBodies">Number of bodies in the current snapshot. Staying at 0 with Auto Capture enabled means no body matching the filters has existed yet.</param>
    /// <param name="simulation">The simulation to snapshot, from a SimulationSettings or GetSimulation node.</param>
    /// <param name="capture">Takes a new snapshot of the poses and velocities of all bodies matching the filters. Connect a Bang. The filter pins are read at this moment; changing them later does not re-capture.</param>
    /// <param name="reset">Restores all captured bodies to their snapshot state. Connect a Bang. Restores the captured set as is: the filters are NOT re-checked, a body whose layer or group changed since the capture is still restored.</param>
    /// <param name="autoCapture">Automatically captures the first frame in which at least one body matching the filters exists: the capture is attempted every frame and keeps retrying while it catches nothing, then stays off. A manual Capture bang also ends the automatic attempts. With filters that never match anything the attempts continue indefinitely (cheap, but watch Captured Bodies staying at 0).</param>
    /// <param name="collisionMask">Which collision layers are captured, default all layers. A body is captured when its layer is contained in this mask AND its group passes Collision Group Id, both filters must agree. Read when a capture happens (bang or auto), never at reset.</param>
    /// <param name="collisionGroupId">Only bodies whose collision group Id matches are captured, combined with Collision Mask (both must agree). -1 = all groups. Note that 0 selects exactly the bodies WITHOUT a group, since Id 0 is the no group default; this is why -1 is the off value. The group's index values are ignored on purpose: members of one group (for example chain links) carry different indices by design, the Id alone says which set a body belongs to. Read when a capture happens, never at reset.</param>
    public void Update(
        out int capturedBodies,
        SBepu.BepuSimulation? simulation = null,
        bool capture = false,
        bool reset = false,
        bool autoCapture = true,
        [Pin(Visibility = PinVisibility.Optional)] SBepu.CollisionMask collisionMask = SBepu.CollisionMask.Everything,
        [Pin(Visibility = PinVisibility.Optional)] int collisionGroupId = -1)
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

        if (captureEdge)
        {
            // A manual bang always counts, even when it catches nothing: it expresses
            // intent and also consumes a pending auto capture.
            Capture(simulation, collisionMask, collisionGroupId);
            _hasCaptured = true;
        }
        else if (autoCapture && !_hasCaptured)
        {
            // The attempt itself is the check (no separate HasBodies twin that would have
            // to duplicate the filter logic): an empty result leaves _hasCaptured false,
            // so the next frame tries again. Empty attempts allocate nothing.
            Capture(simulation, collisionMask, collisionGroupId);
            _hasCaptured = _snapshot.Count > 0;
        }

        if (resetEdge)
            Restore();

        capturedBodies = _snapshot.Count;
    }

    private void Capture(SBepu.BepuSimulation simulation, SBepu.CollisionMask collisionMask, int collisionGroupId)
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
                if (((uint)collisionMask & (1u << (int)component.CollisionLayer)) == 0)
                    continue;
                if (collisionGroupId >= 0 && component.CollisionGroup.Id != collisionGroupId)
                    continue;
                _snapshot.Add(new BodySnapshot(
                    component,
                    component.Position,
                    component.Orientation,
                    component.LinearVelocity,
                    component.AngularVelocity));
            }
        }
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
