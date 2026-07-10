using System.ComponentModel;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using VL.Core;
using VL.Core.Import;
using VL.Lib.Basics.Resources;
using VL.Lib.Collections;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
using SBepu = global::Stride.BepuPhysics;

namespace VL.Stride.BepuPhysics.Simulation;

/// <summary>
/// Configures the Bepu physics simulation (gravity, timing, solver, collision matrix)
/// and outputs it for use with query nodes. Uses the same lazily-created simulation the
/// Body/Static components attach to.
/// Use only ONE SimulationSettings node per simulation — several instances write to the
/// same global state and fight each other. To merely read the simulation, use GetSimulation.
/// </summary>
[ProcessNode(Name = "SimulationSettings")]
public class SimulationSettingsNode : IDisposable
{
    private readonly IResourceHandle<Game> _gameHandle = AppHost.Current.Services.GetGameHandle();
    private SBepu.BepuConfiguration? _config;
    private object? _lastMatrix = new(); // sentinel so a connected matrix is applied on first frame
    // Shadow the solver pins against the last PIN value: BepuSimulation itself rewrites
    // Solver.SubstepCount during its soft-start window (boost ×factor, then divide back),
    // so diffing against the live solver value fights the engine and corrupts the soft-start
    // restore (SubstepCount 1/4 = 0 → ArgumentException). PinValue reports the first call as
    // a change, so the pins are authoritative from the first frame like every other pin here.
    private PinValue<int> _velocityIterations;
    private PinValue<int> _subSteps;

    /// <param name="gravity">Global gravity applied to all non-kinematic bodies. Null = (0, -9.8, 0).</param>
    /// <param name="linearDamping">How quickly bodies lose linear velocity over time.</param>
    /// <param name="angularDamping">How quickly bodies lose angular velocity over time.</param>
    /// <param name="timeScale">Scales simulation time; 0.5 = slow motion, 2 = double speed.</param>
    /// <param name="fixedTimeStepSeconds">Length of one physics step in seconds (default 1/60).</param>
    /// <param name="maxStepPerFrame">Upper limit of physics steps per frame to avoid spirals of death; -1 = unlimited.</param>
    /// <param name="parallelUpdate">Runs per-body updates multithreaded.</param>
    /// <param name="usePerBodyAttributes">Enables per-body Gravity flags (slightly more per-body work).</param>
    /// <param name="solverVelocityIterations">Solver velocity iterations per substep; more = stiffer, costlier.</param>
    /// <param name="solverSubSteps">Solver substeps per physics step; more = more accurate stacks and joints.</param>
    /// <param name="collisionMatrix">Per-layer collision masks: element N = mask of layers that layer N collides with. Layers beyond the spread's count keep their current value; disconnecting does not reset the matrix.</param>
    /// <param name="enabled">Pauses the whole simulation when false.</param>
    /// <param name="simulationIndex">Which simulation to configure. Currently only simulation 0 exists — additional simulations cannot yet be created, any other index outputs null.</param>
    [return: Pin(Name = "Output")]
    public SBepu.BepuSimulation? Update(
        [DefaultValue("0.0, -9.8, 0.0")] Vector3 gravity,
        float linearDamping = 0.05f,
        float angularDamping = 0.05f,
        float timeScale = 1f,
        double fixedTimeStepSeconds = 1.0 / 60,
        int maxStepPerFrame = 3,
        bool parallelUpdate = true,
        bool usePerBodyAttributes = false,
        int solverVelocityIterations = 8,
        int solverSubSteps = 1,
        [Pin(Visibility = PinVisibility.Optional)] Spread<SBepu.CollisionMask>? collisionMatrix = null,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] int simulationIndex = 0)
    {
        // Same path the engine's own processors take — creates the configuration,
        // a default simulation and the PhysicsGameSystem on first use.
        BepuSettingsBootstrap.EnsureConfigured();
        _config ??= _gameHandle.Resource.Services.GetOrCreate<SBepu.BepuConfiguration>();

        if (simulationIndex < 0 || simulationIndex >= _config.BepuSimulations.Count)
            return null;
        var sim = _config.BepuSimulations[simulationIndex];


        if (sim.PoseGravity != gravity)
            sim.PoseGravity = gravity;
        if (sim.PoseLinearDamping != linearDamping)
            sim.PoseLinearDamping = linearDamping;
        if (sim.PoseAngularDamping != angularDamping)
            sim.PoseAngularDamping = angularDamping;
        if (sim.TimeScale != timeScale)
            sim.TimeScale = timeScale;
        if (sim.FixedTimeStepSeconds != fixedTimeStepSeconds)
            sim.FixedTimeStepSeconds = fixedTimeStepSeconds;
        if (sim.MaxStepPerFrame != maxStepPerFrame)
            sim.MaxStepPerFrame = maxStepPerFrame;
        if (sim.ParallelUpdate != parallelUpdate)
            sim.ParallelUpdate = parallelUpdate;
        if (sim.UsePerBodyAttributes != usePerBodyAttributes)
            sim.UsePerBodyAttributes = usePerBodyAttributes;
        if (sim.Enabled != enabled)
            sim.Enabled = enabled;

        // SolverIteration/SolverSubStep are init-only on the Stride wrapper,
        // but Bepu's Solver exposes them mutable — apply live through it (on pin change only).
        if (_velocityIterations.Changed(solverVelocityIterations) && solverVelocityIterations > 0)
            sim.Simulation.Solver.VelocityIterationCount = solverVelocityIterations;
        if (_subSteps.Changed(solverSubSteps) && solverSubSteps > 0)
        {
            sim.Simulation.Solver.SubstepCount = solverSubSteps;
            // Restart the soft-start window so the engine's boost/restore math stays consistent.
            sim.ResetSoftStart();
        }

        if (!ReferenceEquals(_lastMatrix, collisionMatrix))
        {
            _lastMatrix = collisionMatrix;
            if (collisionMatrix is not null && collisionMatrix.Count > 0)
            {
                for (int i = 0; i < collisionMatrix.Count && i < 32; i++)
                    sim.CollisionMatrix.Set((SBepu.CollisionLayer)i, collisionMatrix[i]);
            }
        }

        return sim;
    }

    public void Dispose() => _gameHandle.Dispose();
}

/// <summary>
/// Outputs the running Bepu simulation (for query nodes) without changing its settings.
/// </summary>
[ProcessNode(Name = "GetSimulation")]
public class GetSimulationNode : IDisposable
{
    private readonly IResourceHandle<Game> _gameHandle = AppHost.Current.Services.GetGameHandle();
    private SBepu.BepuConfiguration? _config;

    /// <param name="simulationIndex">Which simulation to fetch. Currently only simulation 0 exists — additional simulations cannot yet be created, any other index outputs null.</param>
    [return: Pin(Name = "Output")]
    public SBepu.BepuSimulation? Update(int simulationIndex = 0)
    {
        BepuSettingsBootstrap.EnsureConfigured();
        _config ??= _gameHandle.Resource.Services.GetOrCreate<SBepu.BepuConfiguration>();
        if (simulationIndex < 0 || simulationIndex >= _config.BepuSimulations.Count)
            return null;
        return _config.BepuSimulations[simulationIndex];
    }

    public void Dispose() => _gameHandle.Dispose();
}
