using VL.Core.Import;
using VL.Lib.Collections;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
using SBepu = global::Stride.BepuPhysics;
using SColliders = global::Stride.BepuPhysics.Definitions.Colliders;
using SContacts = global::Stride.BepuPhysics.Definitions.Contacts;
using SDefinitions = global::Stride.BepuPhysics.Definitions;

namespace VL.Stride.BepuPhysics.Components;

/// <summary>
/// Immovable collision geometry (ground, walls, level geometry).
/// Connect the output to an Entity; the entity's transform places the static shape.
/// Connect one or more collider shapes; each shape instance must only be used by one collidable.
/// </summary>
[ProcessNode(Name = "Static")]
public class StaticNode
{
    private readonly ColliderInput _colliderInput = new();
    private readonly SBepu.StaticComponent _component;

    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.CollisionLayer> _collisionLayer;
    private PinValue<SDefinitions.CollisionGroup> _collisionGroup;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<float> _frictionCoefficient;
    private PinValue<float> _maximumRecoveryVelocity;
    private PinValue<SContacts.IContactHandler?> _contactHandler;

    public StaticNode()
    {
        BepuSettingsBootstrap.EnsureConfigured();
        _component = new SBepu.StaticComponent { Collider = new SColliders.EmptyCollider() };
    }

    /// <param name="colliders">Collision shapes forming this static (combined into one rigid compound). Each shape instance must only be used by one collidable.</param>
    /// <param name="collisionLayer">The collision layer of this static (0..31); pair filtering is configured via the SimulationSettings collision matrix.</param>
    /// <param name="collisionGroup">Fine grained filter on top of the collision layer: collidables sharing the same non zero Id ignore each other while their indices differ by less than two. Create with the CollisionGroup operation. Null = no group.</param>
    /// <param name="springFrequency">Contact spring stiffness in Hz — how hard contacts push overlapping bodies apart.</param>
    /// <param name="springDampingRatio">Contact spring damping; 1 = critical damping, higher values settle contacts more stiffly.</param>
    /// <param name="frictionCoefficient">Surface friction; 0 = frictionless, 1 = rough. Combined with the other collidable's coefficient on contact.</param>
    /// <param name="maximumRecoveryVelocity">Upper limit for the velocity used to push overlapping bodies apart; lower values soften deep-contact pops.</param>
    /// <param name="contactHandler">Connect a ContactEvents node's output here to receive contact begin/touch/end events for this static.</param>
    /// <param name="colliderOverride">Advanced: use a MeshCollider or EmptyCollider instead of the Colliders shapes.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    /// <returns>The static component — connect to an Entity's Components input.</returns>
    [return: Pin(Name = "Output")]
    public SBepu.StaticComponent Update(
        [Pin(PinGroupKind = PinGroupKind.Collection, PinGroupDefaultCount = 1)] Spread<SColliders.ColliderBase?>? colliders = null,
        SBepu.CollisionLayer collisionLayer = SBepu.CollisionLayer.Layer0,
        [Pin(Visibility = PinVisibility.Optional)] SDefinitions.CollisionGroup? collisionGroup = null,
        float springFrequency = 30f,
        float springDampingRatio = 3f,
        float frictionCoefficient = 1f,
        float maximumRecoveryVelocity = 1000f,
        SContacts.IContactHandler? contactHandler = null,
        [Pin(Visibility = PinVisibility.Optional)] SColliders.ICollider? colliderOverride = null,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        var effectiveCollider = _colliderInput.Resolve(colliders, colliderOverride);
        if (!ReferenceEquals(_component.Collider, effectiveCollider))
            _component.Collider = effectiveCollider;

        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_collisionLayer.Changed(collisionLayer) | reapplyInputs)
            _component.CollisionLayer = collisionLayer;
        var effectiveGroup = collisionGroup ?? default;
        if (_collisionGroup.Changed(effectiveGroup) | reapplyInputs)
            _component.CollisionGroup = effectiveGroup;
        if (_springFrequency.Changed(springFrequency) | reapplyInputs)
            _component.SpringFrequency = springFrequency;
        if (_springDampingRatio.Changed(springDampingRatio) | reapplyInputs)
            _component.SpringDampingRatio = springDampingRatio;
        if (_frictionCoefficient.Changed(frictionCoefficient) | reapplyInputs)
            _component.FrictionCoefficient = frictionCoefficient;
        if (_maximumRecoveryVelocity.Changed(maximumRecoveryVelocity) | reapplyInputs)
            _component.MaximumRecoveryVelocity = maximumRecoveryVelocity;
        if (_contactHandler.Changed(contactHandler) | reapplyInputs)
            _component.ContactEventHandler = contactHandler;

        return _component;
    }
}
