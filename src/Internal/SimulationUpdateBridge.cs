using System.Reactive.Subjects;
using Stride.Core;
using Stride.Engine;
using VL.Stride.BepuPhysics.Simulation;
using SBepu = global::Stride.BepuPhysics;
using SComponents = global::Stride.BepuPhysics.Components;

namespace VL.Stride.BepuPhysics.Internal;

/// <summary>
/// Entity component implementing ISimulationUpdate: the engine's SimUpdateProcessor picks it
/// up when it is attached to a scene entity and calls the two step methods around every
/// physics step. Buffers step statistics between node updates and additionally pushes each
/// step as observable notifications. Physics steps run on the main thread (inside the game
/// system update), so plain fields without locking are sufficient and OnNext fires on the
/// main thread.
/// </summary>
[DataContract]
internal sealed class SimulationUpdateBridge : EntityComponent, SComponents.ISimulationUpdate, IDisposable
{
    private readonly Subject<SimulationStepInfo> _update = new();
    private readonly Subject<SimulationStepInfo> _afterUpdate = new();
    private int _stepsSinceTake;
    private long _stepCount; // long: an int wraps after ~414 days at 60 steps per second
    private float _lastTimeStep;

    public IObservable<SimulationStepInfo> UpdateObservable => _update;
    public IObservable<SimulationStepInfo> AfterUpdateObservable => _afterUpdate;

    public void SimulationUpdate(SBepu.BepuSimulation simulation, float simTimeStep)
    {
        _stepsSinceTake++;
        _stepCount++;
        _lastTimeStep = simTimeStep;
        _update.OnNext(new SimulationStepInfo(simulation, simTimeStep));
    }

    public void AfterSimulationUpdate(SBepu.BepuSimulation simulation, float simTimeStep)
    {
        _afterUpdate.OnNext(new SimulationStepInfo(simulation, simTimeStep));
    }

    public void TakeFrame(out int stepsLastFrame, out long stepCount, out float lastTimeStep)
    {
        stepsLastFrame = _stepsSinceTake;
        _stepsSinceTake = 0;
        stepCount = _stepCount;
        lastTimeStep = _lastTimeStep;
    }

    public void Dispose()
    {
        _update.OnCompleted();
        _afterUpdate.OnCompleted();
        _update.Dispose();
        _afterUpdate.Dispose();
    }
}
