using Stride.Engine;
using VL.Core.Import;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
using SBepu = global::Stride.BepuPhysics;

namespace VL.Stride.BepuPhysics.Simulation;

/// <summary>
/// One physics simulation step.
/// </summary>
/// <param name="Simulation">The simulation that stepped.</param>
/// <param name="TimeStep">Duration of the step in seconds.</param>
public readonly record struct SimulationStepInfo(
    SBepu.BepuSimulation? Simulation,
    float TimeStep)
{
    // The properties below restate the primary constructor parameters on purpose: only
    // property-level XML docs become pin tooltips in vvvv. Do not "clean them up".

    /// <summary>The simulation that stepped.</summary>
    public SBepu.BepuSimulation? Simulation { get; init; } = Simulation;

    /// <summary>Duration of the step in seconds.</summary>
    public float TimeStep { get; init; } = TimeStep;

    /// <summary>Splits the simulation step into its parts.</summary>
    /// <param name="simulation">The simulation that stepped.</param>
    /// <param name="timeStep">Duration of the step in seconds.</param>
    /// <param name="stepInfo">The unsplit step. Hidden pin whose only purpose is to give this node a different signature than the auto generated Split.</param>
    public void Split(
        out SBepu.BepuSimulation? simulation,
        out float timeStep,
        [Pin(Visibility = PinVisibility.Hidden)] out SimulationStepInfo stepInfo)
    {
        simulation = Simulation;
        timeStep = TimeStep;
        stepInfo = this;
    }
}

/// <summary>
/// Reports every physics simulation step. Connect the Output to any Entity in the scene
/// (the hook only runs while it is part of the scene). Use the observables with reactive
/// regions (ForEach (Reactive), ...) to apply forces or read state once per physics step,
/// independent of the display frame rate. Note: several steps can occur in one frame
/// (and none while the simulation is paused), see Steps Last Frame.
/// </summary>
[ProcessNode(Name = "SimulationUpdate")]
public class SimulationUpdateNode : IDisposable
{
    private readonly SimulationUpdateBridge _bridge = new();

    public SimulationUpdateNode()
    {
        BepuSettingsBootstrap.EnsureConfigured();
    }

    /// <param name="stepsLastFrame">Number of physics steps since the previous frame (0 while paused or when the frame rate outruns the fixed time step).</param>
    /// <param name="stepCount">Total number of physics steps since this node was created.</param>
    /// <param name="timeStep">Duration of the last step in seconds.</param>
    /// <param name="onSimulationUpdate">Observable notification fired before each physics step, for use with reactive nodes.</param>
    /// <param name="onAfterSimulationUpdate">Observable notification fired after each physics step, for use with reactive nodes.</param>
    /// <returns>The hook component, connect to any Entity's Components input.</returns>
    [return: Pin(Name = "Output")]
    public EntityComponent Update(
        out int stepsLastFrame,
        out long stepCount,
        out float timeStep,
        out IObservable<SimulationStepInfo> onSimulationUpdate,
        out IObservable<SimulationStepInfo> onAfterSimulationUpdate)
    {
        _bridge.TakeFrame(out stepsLastFrame, out stepCount, out timeStep);
        onSimulationUpdate = _bridge.UpdateObservable;
        onAfterSimulationUpdate = _bridge.AfterUpdateObservable;
        return _bridge;
    }

    public void Dispose() => _bridge.Dispose();
}
