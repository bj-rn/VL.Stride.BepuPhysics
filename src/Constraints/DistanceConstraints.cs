using Stride.Core.Mathematics;
using VL.Core.Import;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
using SBepu = global::Stride.BepuPhysics;
using SConstraints = global::Stride.BepuPhysics.Constraints;

namespace VL.Stride.BepuPhysics.Constraints;

/// <summary>Keeps the centers of two bodies at a target distance.</summary>
[ProcessNode(Name = "CenterDistance")]
public class CenterDistanceNode
{
    private readonly SConstraints.CenterDistanceConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<float> _targetDistance;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="targetDistance">Distance to maintain between the two body centers.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.CenterDistanceConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA = null,
        SBepu.BodyComponent? bodyB = null,
        float targetDistance = 1f,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_targetDistance.Changed(targetDistance) | reapplyInputs) _c.TargetDistance = targetDistance;
        if (_springFrequency.Changed(springFrequency) | reapplyInputs) _c.SpringFrequency = springFrequency;
        if (_springDampingRatio.Changed(springDampingRatio) | reapplyInputs) _c.SpringDampingRatio = springDampingRatio;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Keeps the distance between the centers of two bodies within a range.</summary>
[ProcessNode(Name = "CenterDistanceLimit")]
public class CenterDistanceLimitNode
{
    private readonly SConstraints.CenterDistanceLimitConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<float> _minimumDistance;
    private PinValue<float> _maximumDistance;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="minimumDistance">Smallest allowed distance between the two body centers.</param>
    /// <param name="maximumDistance">Largest allowed distance between the two body centers.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.CenterDistanceLimitConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA = null,
        SBepu.BodyComponent? bodyB = null,
        float minimumDistance = 0f,
        float maximumDistance = 1f,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_minimumDistance.Changed(minimumDistance) | reapplyInputs) _c.MinimumDistance = minimumDistance;
        if (_maximumDistance.Changed(maximumDistance) | reapplyInputs) _c.MaximumDistance = maximumDistance;
        if (_springFrequency.Changed(springFrequency) | reapplyInputs) _c.SpringFrequency = springFrequency;
        if (_springDampingRatio.Changed(springDampingRatio) | reapplyInputs) _c.SpringDampingRatio = springDampingRatio;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Keeps the distance between two anchor points within a range.</summary>
[ProcessNode(Name = "DistanceLimit")]
public class DistanceLimitNode
{
    private readonly SConstraints.DistanceLimitConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _localOffsetA;
    private PinValue<Vector3> _localOffsetB;
    private PinValue<float> _minimumDistance;
    private PinValue<float> _maximumDistance;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffsetA">Anchor point relative to the center of body A.</param>
    /// <param name="localOffsetB">Anchor point relative to the center of body B.</param>
    /// <param name="minimumDistance">Smallest allowed distance between the two anchor points.</param>
    /// <param name="maximumDistance">Largest allowed distance between the two anchor points.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.DistanceLimitConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA = null,
        SBepu.BodyComponent? bodyB = null,
        Vector3 localOffsetA = default,
        Vector3 localOffsetB = default,
        float minimumDistance = 0f,
        float maximumDistance = 1f,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_localOffsetA.Changed(localOffsetA) | reapplyInputs) _c.LocalOffsetA = localOffsetA;
        if (_localOffsetB.Changed(localOffsetB) | reapplyInputs) _c.LocalOffsetB = localOffsetB;
        if (_minimumDistance.Changed(minimumDistance) | reapplyInputs) _c.MinimumDistance = minimumDistance;
        if (_maximumDistance.Changed(maximumDistance) | reapplyInputs) _c.MaximumDistance = maximumDistance;
        if (_springFrequency.Changed(springFrequency) | reapplyInputs) _c.SpringFrequency = springFrequency;
        if (_springDampingRatio.Changed(springDampingRatio) | reapplyInputs) _c.SpringDampingRatio = springDampingRatio;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Servos the distance between two anchor points towards a target.</summary>
[ProcessNode(Name = "DistanceServo")]
public class DistanceServoNode
{
    private readonly SConstraints.DistanceServoConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _localOffsetA;
    private PinValue<Vector3> _localOffsetB;
    private PinValue<float> _targetDistance;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<float> _servoMaximumSpeed;
    private PinValue<float> _servoBaseSpeed;
    private PinValue<float> _servoMaximumForce;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffsetA">Anchor point relative to the center of body A.</param>
    /// <param name="localOffsetB">Anchor point relative to the center of body B.</param>
    /// <param name="targetDistance">Distance between the two anchor points the servo drives towards.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="servoMaximumSpeed">Maximum speed the servo may use to approach the target.</param>
    /// <param name="servoBaseSpeed">Minimum speed used while correcting remaining error.</param>
    /// <param name="servoMaximumForce">Maximum force the servo may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.DistanceServoConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA = null,
        SBepu.BodyComponent? bodyB = null,
        Vector3 localOffsetA = default,
        Vector3 localOffsetB = default,
        float targetDistance = 1f,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        float servoMaximumSpeed = 10f,
        float servoBaseSpeed = 1f,
        float servoMaximumForce = 1000f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_localOffsetA.Changed(localOffsetA) | reapplyInputs) _c.LocalOffsetA = localOffsetA;
        if (_localOffsetB.Changed(localOffsetB) | reapplyInputs) _c.LocalOffsetB = localOffsetB;
        if (_targetDistance.Changed(targetDistance) | reapplyInputs) _c.TargetDistance = targetDistance;
        if (_springFrequency.Changed(springFrequency) | reapplyInputs) _c.SpringFrequency = springFrequency;
        if (_springDampingRatio.Changed(springDampingRatio) | reapplyInputs) _c.SpringDampingRatio = springDampingRatio;
        if (_servoMaximumSpeed.Changed(servoMaximumSpeed) | reapplyInputs) _c.ServoMaximumSpeed = servoMaximumSpeed;
        if (_servoBaseSpeed.Changed(servoBaseSpeed) | reapplyInputs) _c.ServoBaseSpeed = servoBaseSpeed;
        if (_servoMaximumForce.Changed(servoMaximumForce) | reapplyInputs) _c.ServoMaximumForce = servoMaximumForce;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}
