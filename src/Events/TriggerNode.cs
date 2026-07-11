using VL.Core.Import;
using VL.Lib.Collections;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
using SBepu = global::Stride.BepuPhysics;
using SContacts = global::Stride.BepuPhysics.Definitions.Contacts;

namespace VL.Stride.BepuPhysics.Events;

/// <summary>
/// One trigger event: a collidable entered or exited a trigger volume.
/// </summary>
/// <param name="Source">The trigger collidable the handler is attached to.</param>
/// <param name="Other">The collidable that entered or exited the trigger.</param>
public readonly record struct TriggerInfo(
    SBepu.CollidableComponent? Source,
    SBepu.CollidableComponent? Other)
{
    // The properties below restate the primary constructor parameters on purpose: only
    // property-level XML docs become pin tooltips in vvvv. Do not "clean them up".

    /// <summary>The trigger collidable the handler is attached to.</summary>
    public SBepu.CollidableComponent? Source { get; init; } = Source;

    /// <summary>The collidable that entered or exited the trigger.</summary>
    public SBepu.CollidableComponent? Other { get; init; } = Other;

    /// <summary>Splits the trigger event into its parts.</summary>
    /// <param name="source">The trigger collidable the handler is attached to.</param>
    /// <param name="other">The collidable that entered or exited the trigger.</param>
    /// <param name="triggerInfo">The unsplit trigger event. Hidden pin whose only purpose is to give this node a different signature than the auto generated Split.</param>
    public void Split(
        out SBepu.CollidableComponent? source,
        out SBepu.CollidableComponent? other,
        [Pin(Visibility = PinVisibility.Hidden)] out TriggerInfo triggerInfo)
    {
        source = Source;
        other = Other;
        triggerInfo = this;
    }
}

/// <summary>
/// Turns the collidables its handler output is connected to into trigger volumes: they detect
/// overlap but produce no collision response, others pass through. Connect the Output to the
/// ContactHandler pin of Body/Static nodes, then read which collidables entered or exited.
/// For contact points, depths and solid collision use ContactEvents instead.
/// Note: events only fire on physics steps, nothing fires while the simulation is paused.
/// </summary>
[ProcessNode(Name = "Trigger")]
public class TriggerNode : IDisposable
{
    private readonly BufferingTriggerHandler _handler = new();

    /// <param name="entered">Collidables that started overlapping a trigger this frame.</param>
    /// <param name="exited">Collidables that stopped overlapping a trigger this frame.</param>
    /// <param name="anyEntered">True in frames where anything entered.</param>
    /// <param name="anyExited">True in frames where anything exited.</param>
    /// <param name="onEntered">Observable notification per entering collidable, for use with reactive nodes (ForEach (Reactive), HoldLatest, ...).</param>
    /// <param name="onExited">Observable notification per exiting collidable, for use with reactive nodes.</param>
    /// <returns>The trigger handler, connect to the ContactHandler pin of Body/Static nodes.</returns>
    [return: Pin(Name = "Output")]
    public SContacts.IContactHandler Update(
        out Spread<TriggerInfo> entered,
        out Spread<TriggerInfo> exited,
        out bool anyEntered,
        out bool anyExited,
        out IObservable<TriggerInfo> onEntered,
        out IObservable<TriggerInfo> onExited)
    {
        _handler.TakeFrame(out entered, out exited);
        anyEntered = entered.Count > 0;
        anyExited = exited.Count > 0;
        onEntered = _handler.EnteredObservable;
        onExited = _handler.ExitedObservable;
        return _handler;
    }

    public void Dispose() => _handler.Dispose();
}
