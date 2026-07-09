using Stride.Core.Mathematics;
using VL.Core.Import;
using VL.Lib.Collections;
using VL.Stride.BepuPhysics.Internal;
using SBepu = global::Stride.BepuPhysics;
using SContacts = global::Stride.BepuPhysics.Definitions.Contacts;

namespace VL.Stride.BepuPhysics;

/// <summary>
/// One contact point between two collidables.
/// </summary>
public readonly record struct ContactInfo(
    SBepu.CollidableComponent? Source,
    SBepu.CollidableComponent? Other,
    Vector3 Point,
    Vector3 Normal,
    float Depth);

/// <summary>
/// Collects contact events for the collidables its handler output is connected to.
/// Connect the Handler output to the ContactHandler pin of Body/Static nodes, then either
/// read the per-frame contact spreads or use the observable outputs with VL's reactive nodes.
/// </summary>
[ProcessNode(Name = "ContactEvents")]
public class ContactEventsNode : IDisposable
{
    private readonly BufferingContactHandler _handler = new();

    /// <param name="started">Contacts that began touching this frame.</param>
    /// <param name="touching">Contacts currently touching (fires every frame while in contact).</param>
    /// <param name="stopped">Contacts that stopped touching this frame.</param>
    /// <param name="anyStarted">True in frames where any contact began.</param>
    /// <param name="anyStopped">True in frames where any contact ended.</param>
    /// <param name="onStarted">Observable notification per contact that began touching — for use with reactive nodes (ForEach (Reactive), HoldLatest, ...).</param>
    /// <param name="onStopped">Observable notification per contact that stopped touching — for use with reactive nodes.</param>
    /// <param name="noContactResponse">When true, collidables using this handler let others pass through (trigger volumes).</param>
    [return: Pin(Name = "Output")]
    public SContacts.IContactHandler Update(
        out Spread<ContactInfo> started,
        out Spread<ContactInfo> touching,
        out Spread<ContactInfo> stopped,
        out bool anyStarted,
        out bool anyStopped,
        out IObservable<ContactInfo> onStarted,
        out IObservable<ContactInfo> onStopped,
        bool noContactResponse = false)
    {
        _handler.NoContactResponse = noContactResponse;
        _handler.TakeFrame(out started, out touching, out stopped);
        anyStarted = started.Count > 0;
        anyStopped = stopped.Count > 0;
        onStarted = _handler.StartedObservable;
        onStopped = _handler.StoppedObservable;
        return _handler;
    }

    public void Dispose() => _handler.Dispose();
}
