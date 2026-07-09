using BepuPhysics.CollisionDetection;
using VL.Lib.Collections;
using SContacts = global::Stride.BepuPhysics.Definitions.Contacts;

namespace VL.Stride.BepuPhysics.Internal;

/// <summary>
/// IContactHandler implementation that buffers events between node updates.
/// Physics steps run on the main thread (inside the game system update),
/// so plain lists without locking are sufficient.
/// </summary>
internal sealed class BufferingContactHandler : SContacts.IContactHandler
{
    private readonly SpreadBuilder<ContactInfo> _started = new();
    private readonly SpreadBuilder<ContactInfo> _touching = new();
    private readonly SpreadBuilder<ContactInfo> _stopped = new();

    private Spread<ContactInfo> _lastStarted = Spread<ContactInfo>.Empty;
    private Spread<ContactInfo> _lastTouching = Spread<ContactInfo>.Empty;
    private Spread<ContactInfo> _lastStopped = Spread<ContactInfo>.Empty;

    public bool NoContactResponse { get; set; }

    public void TakeFrame(out Spread<ContactInfo> started, out Spread<ContactInfo> touching, out Spread<ContactInfo> stopped)
    {
        // Only build new spreads when events occurred; otherwise reuse the cached empty/last-frame instances.
        started = _lastStarted = Take(_started, _lastStarted);
        touching = _lastTouching = Take(_touching, _lastTouching);
        stopped = _lastStopped = Take(_stopped, _lastStopped);
    }

    private static Spread<ContactInfo> Take(SpreadBuilder<ContactInfo> builder, Spread<ContactInfo> last)
    {
        if (builder.Count == 0)
            return last.Count == 0 ? last : Spread<ContactInfo>.Empty;
        var result = builder.ToSpread();
        builder.Clear();
        return result;
    }

    void SContacts.IContactHandler.OnStartedTouching<TManifold>(SContacts.Contacts<TManifold> contacts)
        => Collect(contacts, _started, pairOnlyFallback: true);

    void SContacts.IContactHandler.OnTouching<TManifold>(SContacts.Contacts<TManifold> contacts)
        => Collect(contacts, _touching, pairOnlyFallback: false);

    void SContacts.IContactHandler.OnStoppedTouching<TManifold>(SContacts.Contacts<TManifold> contacts)
        => Collect(contacts, _stopped, pairOnlyFallback: true);

    private static void Collect<TManifold>(SContacts.Contacts<TManifold> contacts, SpreadBuilder<ContactInfo> target, bool pairOnlyFallback)
        where TManifold : unmanaged, IContactManifold<TManifold>
    {
        var any = false;
        foreach (var contact in contacts)
        {
            any = true;
            target.Add(new ContactInfo(contacts.EventSource, contacts.Other, contact.Point, contact.Normal, contact.Depth));
        }
        // Started/stopped events can fire with no penetrating contact points — still report the pair.
        if (!any && pairOnlyFallback)
            target.Add(new ContactInfo(contacts.EventSource, contacts.Other, default, default, 0f));
    }
}
