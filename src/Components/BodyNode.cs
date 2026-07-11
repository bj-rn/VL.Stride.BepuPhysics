using Stride.Core.Mathematics;
using VL.Core.Import;
using VL.Lib.Collections;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
using SBepu = global::Stride.BepuPhysics;
using SColliders = global::Stride.BepuPhysics.Definitions.Colliders;
using SContacts = global::Stride.BepuPhysics.Definitions.Contacts;
using SDefinitions = global::Stride.BepuPhysics.Definitions;

namespace VL.Stride.BepuPhysics;

/// <summary>
/// A dynamic or kinematic rigid body simulated by BepuPhysics.
/// Connect the output to an Entity (like a ModelComponent); the body drives the entity's transform.
/// The entity's transform at attach time defines the initial pose.
/// Connect one or more collider shapes; each shape instance can only be used by one body.
/// </summary>
[ProcessNode(Name = "Body")]
public class BodyNode
{
    private readonly ColliderInput _colliderInput = new();
    private readonly SBepu.BodyComponent _component;
    private Matrix? _lastTeleportTo;
    private bool _teleportPending;

    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<bool> _kinematic;
    private PinValue<SDefinitions.InterpolationMode> _interpolation;
    private PinValue<SBepu.CollisionLayer> _collisionLayer;
    private PinValue<SDefinitions.CollisionGroup> _collisionGroup;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<float> _frictionCoefficient;
    private PinValue<float> _maximumRecoveryVelocity;
    private PinValue<float> _sleepThreshold;
    private PinValue<bool> _gravity;
    private PinValue<global::BepuPhysics.Collidables.ContinuousDetectionMode> _continuousDetection;
    private PinValue<SContacts.IContactHandler?> _contactHandler;

    public BodyNode()
    {
        BepuSettingsBootstrap.EnsureConfigured();
        _component = new SBepu.BodyComponent
        {
            Collider = new SColliders.EmptyCollider(),
            InterpolationMode = SDefinitions.InterpolationMode.Interpolated,
        };
    }

    /// <param name="colliders">Collision shapes forming this body (combined into one rigid compound). Each shape instance can only be used by one body.</param>
    /// <param name="kinematic">When true the body is unaffected by forces and collisions but pushes dynamic bodies away; move it via SetTargetPose or TeleportTo.</param>
    /// <param name="interpolation">Smooths the rendered motion between fixed physics steps. Null = Interpolated (recommended for display-rate rendering).</param>
    /// <param name="collisionLayer">The collision layer of this body (0..31); pair filtering is configured via the SimulationSettings collision matrix. Null = Layer0.</param>
    /// <param name="collisionGroup">Fine grained filter on top of the collision layer: collidables sharing the same non zero Id ignore each other while their indices differ by less than two. Create with the CollisionGroup operation. Null = no group.</param>
    /// <param name="springFrequency">Contact spring stiffness in Hz — how hard contacts push overlapping bodies apart.</param>
    /// <param name="springDampingRatio">Contact spring damping; 1 = critical damping, higher values settle contacts more stiffly.</param>
    /// <param name="frictionCoefficient">Surface friction; 0 = frictionless, 1 = rough. Combined with the other collidable's coefficient on contact.</param>
    /// <param name="maximumRecoveryVelocity">Upper limit for the velocity used to push overlapping bodies apart; lower values soften deep-contact pops.</param>
    /// <param name="sleepThreshold">Velocity below which the body becomes a sleep candidate; -1 disables sleeping.</param>
    /// <param name="gravity">Whether gravity affects this body. Only evaluated when UsePerBodyAttributes is enabled on the simulation.</param>
    /// <param name="continuousDetection">Continuous collision detection mode. Null = Discrete; use Continuous for fast bodies that would tunnel through thin geometry.</param>
    /// <param name="contactHandler">Connect a ContactEvents node's output here to receive contact begin/touch/end events for this body.</param>
    /// <param name="colliderOverride">Advanced: use a MeshCollider or EmptyCollider instead of the Colliders shapes.</param>
    /// <param name="teleportTo">Places the body at this pose whenever the value CHANGES (a constant matrix = initial pose only). Do not wire a transformation into the entity instead.</param>
    /// <param name="resetPose">While true, re-teleports the body to the TeleportTo pose and zeroes its velocities. Connect a Bang.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    /// <returns>The body component — connect to an Entity's Components input.</returns>
    [return: Pin(Name = "Output")]
    public SBepu.BodyComponent Update(
        [Pin(PinGroupKind = PinGroupKind.Collection, PinGroupDefaultCount = 1)] Spread<SColliders.ColliderBase?>? colliders = null,
        bool kinematic = false,
        SDefinitions.InterpolationMode? interpolation = null,
        SBepu.CollisionLayer? collisionLayer = null,
        [Pin(Visibility = PinVisibility.Optional)] SDefinitions.CollisionGroup? collisionGroup = null,
        float springFrequency = 30f,
        float springDampingRatio = 3f,
        float frictionCoefficient = 1f,
        float maximumRecoveryVelocity = 1000f,
        float sleepThreshold = 0.01f,
        bool gravity = true,
        ContinuousDetectionKind? continuousDetection = null,
        SContacts.IContactHandler? contactHandler = null,
        [Pin(Visibility = PinVisibility.Optional)] SColliders.ICollider? colliderOverride = null,
        Matrix? teleportTo = null,
        bool resetPose = false,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        var effectiveCollider = _colliderInput.Resolve(colliders, colliderOverride);
        if (!ReferenceEquals(_component.Collider, effectiveCollider))
            _component.Collider = effectiveCollider;

        // null = default: Interpolated (smooth visuals at display rate vs the fixed physics step)
        var effectiveInterpolation = interpolation ?? SDefinitions.InterpolationMode.Interpolated;
        var effectiveLayer = collisionLayer ?? SBepu.CollisionLayer.Layer0;

        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_kinematic.Changed(kinematic) | reapplyInputs)
            _component.Kinematic = kinematic;
        if (_interpolation.Changed(effectiveInterpolation) | reapplyInputs)
            _component.InterpolationMode = effectiveInterpolation;
        if (_collisionLayer.Changed(effectiveLayer) | reapplyInputs)
            _component.CollisionLayer = effectiveLayer;
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
        if (_sleepThreshold.Changed(sleepThreshold) | reapplyInputs)
            _component.SleepThreshold = sleepThreshold;
        if (_gravity.Changed(gravity) | reapplyInputs)
            _component.Gravity = gravity;
        var detectionMode = (global::BepuPhysics.Collidables.ContinuousDetectionMode)(continuousDetection ?? ContinuousDetectionKind.Discrete);
        if (_continuousDetection.Changed(detectionMode) | reapplyInputs)
            _component.ContinuousDetectionMode = detectionMode;
        if (_contactHandler.Changed(contactHandler) | reapplyInputs)
            _component.ContactEventHandler = contactHandler;

        // Places the body whenever the connected matrix CHANGES — a constant matrix therefore
        // only sets the initial pose; afterwards physics owns the transform. (Do NOT wire a
        // transformation into the entity instead: the entity node re-applies it every frame,
        // overriding the physics write-back.) ResetPose re-applies the same pose on demand.
        if (teleportTo != _lastTeleportTo)
        {
            _lastTeleportTo = teleportTo;
            _teleportPending = teleportTo.HasValue;
        }
        if ((_teleportPending || resetPose) && _component.Simulation is not null && _lastTeleportTo.HasValue)
        {
            _lastTeleportTo.Value.Decompose(out _, out Quaternion rotation, out Vector3 position);
            // Wake first — mutating a sleeping body leaves it intermittently frozen.
            _component.Awake = true;
            _component.Teleport(position, rotation);
            _component.LinearVelocity = Vector3.Zero;
            _component.AngularVelocity = Vector3.Zero;
            _teleportPending = false;
        }

        return _component;
    }
}
