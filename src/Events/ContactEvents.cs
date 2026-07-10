using Stride.Core.Mathematics;
using VL.Core.Import;
using VL.Lib.Collections;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
using SBepu = global::Stride.BepuPhysics;
using SContacts = global::Stride.BepuPhysics.Definitions.Contacts;

namespace VL.Stride.BepuPhysics;

/// <summary>
/// One contact point between two collidables.
/// </summary>
/// <param name="Source">The collidable the contact handler is attached to.</param>
/// <param name="Other">The other collidable involved in the contact.</param>
/// <param name="Point">The contact point in world space.</param>
/// <param name="Normal">The contact normal in world space, pointing away from the source.</param>
/// <param name="Depth">Penetration depth at the contact point.</param>
public readonly record struct ContactInfo(
    SBepu.CollidableComponent? Source,
    SBepu.CollidableComponent? Other,
    Vector3 Point,
    Vector3 Normal,
    float Depth)
{
    // The properties below restate the primary constructor parameters on purpose: only
    // property-level XML docs become pin tooltips in vvvv. Do not "clean them up".

    /// <summary>The collidable the contact handler is attached to.</summary>
    public SBepu.CollidableComponent? Source { get; init; } = Source;

    /// <summary>The other collidable involved in the contact.</summary>
    public SBepu.CollidableComponent? Other { get; init; } = Other;

    /// <summary>The contact point in world space.</summary>
    public Vector3 Point { get; init; } = Point;

    /// <summary>The contact normal in world space, pointing away from the source.</summary>
    public Vector3 Normal { get; init; } = Normal;

    /// <summary>Penetration depth at the contact point.</summary>
    public float Depth { get; init; } = Depth;

    /// <summary>Splits the contact into its parts.</summary>
    /// <param name="source">The collidable the contact handler is attached to.</param>
    /// <param name="other">The other collidable involved in the contact.</param>
    /// <param name="point">The contact point in world space.</param>
    /// <param name="normal">The contact normal in world space, pointing away from the source.</param>
    /// <param name="depth">Penetration depth at the contact point.</param>
    /// <param name="contactInfo">The unsplit contact. Hidden pin whose only purpose is to give this node a different signature than the auto generated Split.</param>
    public void Split(
        out SBepu.CollidableComponent? source,
        out SBepu.CollidableComponent? other,
        out Vector3 point,
        out Vector3 normal,
        out float depth,
        [Pin(Visibility = PinVisibility.Hidden)] out ContactInfo contactInfo)
    {
        source = Source;
        other = Other;
        point = Point;
        normal = Normal;
        depth = Depth;
        contactInfo = this;
    }
}

/// <summary>
/// Collects contact events for the collidables its handler output is connected to.
/// Connect the Handler output to the ContactHandler pin of Body/Static nodes, then either
/// read the per-frame contact spreads or use the observable outputs with VL's reactive nodes.
/// Note: events only fire on physics steps, while the simulation is paused (Enabled = false)
/// the Touching output goes empty even though bodies still overlap.
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
    /// <param name="onStarted">Observable notification per contact that began touching, for use with reactive nodes (ForEach (Reactive), HoldLatest, ...).</param>
    /// <param name="onStopped">Observable notification per contact that stopped touching, for use with reactive nodes.</param>
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
