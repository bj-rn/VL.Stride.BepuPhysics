using Stride.Core.Mathematics;
using VL.Core.Import;
using VL.Lib.Collections;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
using SBepu = global::Stride.BepuPhysics;
using SColliders = global::Stride.BepuPhysics.Definitions.Colliders;
using SDefinitions = global::Stride.BepuPhysics.Definitions;

namespace VL.Stride.BepuPhysics;

/// <summary>
/// A physics driven character: a dynamic body that walks and jumps.
/// Connect the output to an Entity (like a ModelComponent); a capsule is the typical collider.
/// Drive it with the Move and TryJump operations and read state with CharacterState.
/// The character handles its own contact events for ground detection, therefore it has no
/// ContactHandler pin. Use Trigger or ContactEvents on the collidables it interacts with instead.
/// </summary>
[ProcessNode(Name = "Character")]
public class CharacterNode
{
    private readonly ColliderInput _colliderInput = new();
    private readonly SBepu.CharacterComponent _component;
    private Matrix? _lastTeleportTo;
    private bool _teleportPending;

    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<float> _speed;
    private PinValue<float> _jumpForce;
    private PinValue<SDefinitions.InterpolationMode> _interpolation;
    private PinValue<SBepu.CollisionLayer> _collisionLayer;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<float> _frictionCoefficient;
    private PinValue<float> _maximumRecoveryVelocity;
    private PinValue<float> _sleepThreshold;
    private PinValue<bool> _gravity;
    private PinValue<global::BepuPhysics.Collidables.ContinuousDetectionMode> _continuousDetection;

    public CharacterNode()
    {
        BepuSettingsBootstrap.EnsureConfigured();
        _component = new SBepu.CharacterComponent
        {
            Collider = new SColliders.EmptyCollider(),
            InterpolationMode = SDefinitions.InterpolationMode.Interpolated,
        };
    }

    /// <param name="colliders">Collision shapes forming this character (combined into one rigid compound), a capsule is typical. Each shape instance can only be used by one collidable.</param>
    /// <param name="speed">Base movement speed in units per second, scales the direction given to Move.</param>
    /// <param name="jumpForce">Force of the impulse applied by TryJump.</param>
    /// <param name="interpolation">Smooths the rendered motion between fixed physics steps. Null = Interpolated (recommended for display-rate rendering).</param>
    /// <param name="collisionLayer">The collision layer of this character (0..31); pair filtering is configured via the SimulationSettings collision matrix. Null = Layer0.</param>
    /// <param name="springFrequency">Contact spring stiffness in Hz, how hard contacts push overlapping bodies apart.</param>
    /// <param name="springDampingRatio">Contact spring damping; 1 = critical damping, higher values settle contacts more stiffly.</param>
    /// <param name="frictionCoefficient">Surface friction; 0 = frictionless, 1 = rough. Combined with the other collidable's coefficient on contact.</param>
    /// <param name="maximumRecoveryVelocity">Upper limit for the velocity used to push overlapping bodies apart; lower values soften deep-contact pops.</param>
    /// <param name="sleepThreshold">Velocity below which the body becomes a sleep candidate. Default -1 keeps the character always awake so it reacts to Move immediately.</param>
    /// <param name="gravity">Whether gravity affects this character. Only evaluated when UsePerBodyAttributes is enabled on the simulation.</param>
    /// <param name="continuousDetection">Continuous collision detection mode. Null = Discrete; use Continuous for fast characters that would tunnel through thin geometry.</param>
    /// <param name="colliderOverride">Advanced: use a MeshCollider or EmptyCollider instead of the Colliders shapes.</param>
    /// <param name="teleportTo">Places the character at this pose whenever the value CHANGES (a constant matrix = initial pose only). Do not wire a transformation into the entity instead.</param>
    /// <param name="resetPose">While true, re-teleports the character to the TeleportTo pose and zeroes its velocities. Connect a Bang.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    /// <returns>The character component, connect to an Entity's Components input.</returns>
    [return: Pin(Name = "Output")]
    public SBepu.CharacterComponent Update(
        [Pin(PinGroupKind = PinGroupKind.Collection, PinGroupDefaultCount = 1)] Spread<SColliders.ColliderBase?>? colliders = null,
        float speed = 10f,
        float jumpForce = 10f,
        SDefinitions.InterpolationMode? interpolation = null,
        SBepu.CollisionLayer? collisionLayer = null,
        float springFrequency = 30f,
        float springDampingRatio = 3f,
        float frictionCoefficient = 1f,
        float maximumRecoveryVelocity = 1000f,
        float sleepThreshold = -1f,
        bool gravity = true,
        ContinuousDetectionKind? continuousDetection = null,
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
        if (_speed.Changed(speed) | reapplyInputs)
            _component.Speed = speed;
        if (_jumpForce.Changed(jumpForce) | reapplyInputs)
            _component.JumpForce = jumpForce;
        if (_interpolation.Changed(effectiveInterpolation) | reapplyInputs)
            _component.InterpolationMode = effectiveInterpolation;
        if (_collisionLayer.Changed(effectiveLayer) | reapplyInputs)
            _component.CollisionLayer = effectiveLayer;
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

        // Places the character whenever the connected matrix CHANGES, a constant matrix therefore
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
            // Wake first, mutating a sleeping body leaves it intermittently frozen.
            _component.Awake = true;
            _component.Teleport(position, rotation);
            _component.LinearVelocity = Vector3.Zero;
            _component.AngularVelocity = Vector3.Zero;
            _teleportPending = false;
        }

        return _component;
    }
}
