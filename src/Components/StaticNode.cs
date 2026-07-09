using VL.Core.Import;
using VL.Lib.Collections;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
using SBepu = global::Stride.BepuPhysics;
using SColliders = global::Stride.BepuPhysics.Definitions.Colliders;
using SContacts = global::Stride.BepuPhysics.Definitions.Contacts;

namespace VL.Stride.BepuPhysics;

/// <summary>
/// Immovable collision geometry (ground, walls, level geometry).
/// Connect the output to an Entity; the entity's transform places the static shape.
/// Connect one or more collider shapes; each shape instance can only be used by one collidable.
/// </summary>
[ProcessNode(Name = "Static")]
public class StaticNode
{
    private readonly ColliderInput _colliderInput = new();
    private readonly SBepu.StaticComponent _component;

    public StaticNode()
    {
        _component = new SBepu.StaticComponent { Collider = new SColliders.EmptyCollider() };
    }

    /// <param name="colliders">Collision shapes forming this static (combined into one rigid compound). Each shape instance can only be used by one collidable.</param>
    /// <param name="collisionLayer">The collision layer of this static (0..31); pair filtering is configured via the SimulationSettings collision matrix. Null = Layer0.</param>
    /// <param name="springFrequency">Contact spring stiffness in Hz — how hard contacts push overlapping bodies apart.</param>
    /// <param name="springDampingRatio">Contact spring damping; 1 = critical damping, higher values settle contacts more stiffly.</param>
    /// <param name="frictionCoefficient">Surface friction; 0 = frictionless, 1 = rough. Combined with the other collidable's coefficient on contact.</param>
    /// <param name="maximumRecoveryVelocity">Upper limit for the velocity used to push overlapping bodies apart; lower values soften deep-contact pops.</param>
    /// <param name="contactHandler">Connect a ContactEvents node's output here to receive contact begin/touch/end events for this static.</param>
    /// <param name="colliderOverride">Advanced: use a MeshCollider or EmptyCollider instead of the Colliders shapes.</param>
    /// <returns>The static component — connect to an Entity's Components input.</returns>
    [return: Pin(Name = "Output")]
    public SBepu.StaticComponent Update(
        [Pin(PinGroupKind = PinGroupKind.Collection, PinGroupDefaultCount = 1)] Spread<SColliders.ColliderBase?>? colliders = null,
        SBepu.CollisionLayer? collisionLayer = null,
        float springFrequency = 30f,
        float springDampingRatio = 3f,
        float frictionCoefficient = 1f,
        float maximumRecoveryVelocity = 1000f,
        SContacts.IContactHandler? contactHandler = null,
        [Pin(Visibility = PinVisibility.Optional)] SColliders.ICollider? colliderOverride = null)
    {
        var effectiveCollider = _colliderInput.Resolve(colliders, colliderOverride);
        if (!ReferenceEquals(_component.Collider, effectiveCollider))
            _component.Collider = effectiveCollider;

        var effectiveLayer = collisionLayer ?? SBepu.CollisionLayer.Layer0;
        if (_component.CollisionLayer != effectiveLayer)
            _component.CollisionLayer = effectiveLayer;
        if (_component.SpringFrequency != springFrequency)
            _component.SpringFrequency = springFrequency;
        if (_component.SpringDampingRatio != springDampingRatio)
            _component.SpringDampingRatio = springDampingRatio;
        if (_component.FrictionCoefficient != frictionCoefficient)
            _component.FrictionCoefficient = frictionCoefficient;
        if (_component.MaximumRecoveryVelocity != maximumRecoveryVelocity)
            _component.MaximumRecoveryVelocity = maximumRecoveryVelocity;
        if (!ReferenceEquals(_component.ContactEventHandler, contactHandler))
            _component.ContactEventHandler = contactHandler;

        return _component;
    }
}
