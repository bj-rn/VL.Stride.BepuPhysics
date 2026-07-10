using System.Reactive.Subjects;
using BepuPhysics.CollisionDetection;
using VL.Lib.Collections;
using SContacts = global::Stride.BepuPhysics.Definitions.Contacts;

namespace VL.Stride.BepuPhysics.Internal;

/// <summary>
/// IContactHandler implementation for trigger volumes: no collision response, buffers
/// enter/exit pair events between node updates and additionally pushes them as observable
/// notifications. Physics steps run on the main thread (inside the game system update),
/// so plain lists without locking are sufficient and OnNext fires on the main thread.
/// </summary>
internal sealed class BufferingTriggerHandler : SContacts.IContactHandler, IDisposable
{
    private readonly SpreadBuilder<TriggerInfo> _entered = new();
    private readonly SpreadBuilder<TriggerInfo> _exited = new();

    private Spread<TriggerInfo> _lastEntered = Spread<TriggerInfo>.Empty;
    private Spread<TriggerInfo> _lastExited = Spread<TriggerInfo>.Empty;

    private readonly Subject<TriggerInfo> _enteredSubject = new();
    private readonly Subject<TriggerInfo> _exitedSubject = new();

    public bool NoContactResponse => true;

    public IObservable<TriggerInfo> EnteredObservable => _enteredSubject;
    public IObservable<TriggerInfo> ExitedObservable => _exitedSubject;

    public void TakeFrame(out Spread<TriggerInfo> entered, out Spread<TriggerInfo> exited)
    {
        // Only build new spreads when events occurred; otherwise reuse the cached empty instances.
        entered = _lastEntered = Take(_entered, _lastEntered);
        exited = _lastExited = Take(_exited, _lastExited);
    }

    private static Spread<TriggerInfo> Take(SpreadBuilder<TriggerInfo> builder, Spread<TriggerInfo> last)
    {
        if (builder.Count == 0)
            return last.Count == 0 ? last : Spread<TriggerInfo>.Empty;
        var result = builder.ToSpread();
        builder.Clear();
        return result;
    }

    void SContacts.IContactHandler.OnStartedTouching<TManifold>(SContacts.Contacts<TManifold> contacts)
        => Collect(contacts, _entered, _enteredSubject);

    void SContacts.IContactHandler.OnStoppedTouching<TManifold>(SContacts.Contacts<TManifold> contacts)
        => Collect(contacts, _exited, _exitedSubject);

    private static void Collect<TManifold>(SContacts.Contacts<TManifold> contacts,
        SpreadBuilder<TriggerInfo> target, Subject<TriggerInfo> subject)
        where TManifold : unmanaged, IContactManifold<TManifold>
    {
        var info = new TriggerInfo(contacts.EventSource, contacts.Other);
        target.Add(info);
        subject.OnNext(info);
    }

    public void Dispose()
    {
        _enteredSubject.OnCompleted();
        _exitedSubject.OnCompleted();
        _enteredSubject.Dispose();
        _exitedSubject.Dispose();
    }
}
