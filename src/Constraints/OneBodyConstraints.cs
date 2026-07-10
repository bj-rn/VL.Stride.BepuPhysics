using System.ComponentModel;
using Stride.Core.Mathematics;
using VL.Core.Import;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
using SBepu = global::Stride.BepuPhysics;
using SConstraints = global::Stride.BepuPhysics.Constraints;

namespace VL.Stride.BepuPhysics.Constraints;

/// <summary>Drives a single body's angular velocity towards a target.</summary>
[ProcessNode(Name = "OneBodyAngularMotor")]
public class OneBodyAngularMotorNode
{
    private readonly SConstraints.OneBodyAngularMotorConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _body;
    private PinValue<Vector3> _targetVelocity;
    private PinValue<float> _motorDamping;
    private PinValue<float> _motorMaximumForce;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="body">The constrained body.</param>
    /// <param name="targetVelocity">Target angular velocity in world space, radians per second.</param>
    /// <param name="motorDamping">How aggressively the motor corrects towards the target velocity.</param>
    /// <param name="motorMaximumForce">Maximum force the motor may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.OneBodyAngularMotorConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? body,
        Vector3 targetVelocity,
        float motorDamping = 10f,
        float motorMaximumForce = 1000f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_body.Changed(body) | reapplyInputs) _c.A = body;
        if (_targetVelocity.Changed(targetVelocity) | reapplyInputs) _c.TargetVelocity = targetVelocity;
        if (_motorDamping.Changed(motorDamping) | reapplyInputs) _c.MotorDamping = motorDamping;
        if (_motorMaximumForce.Changed(motorMaximumForce) | reapplyInputs) _c.MotorMaximumForce = motorMaximumForce;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Servos a single body's orientation towards a target.</summary>
[ProcessNode(Name = "OneBodyAngularServo")]
public class OneBodyAngularServoNode
{
    private readonly SConstraints.OneBodyAngularServoConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _body;
    private PinValue<Quaternion> _targetOrientation;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<float> _servoMaximumSpeed;
    private PinValue<float> _servoBaseSpeed;
    private PinValue<float> _servoMaximumForce;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="body">The constrained body.</param>
    /// <param name="targetOrientation">World-space orientation the servo drives the body towards.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="servoMaximumSpeed">Maximum speed the servo may use to approach the target.</param>
    /// <param name="servoBaseSpeed">Minimum speed used while correcting remaining error.</param>
    /// <param name="servoMaximumForce">Maximum force the servo may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.OneBodyAngularServoConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? body,
        Quaternion targetOrientation,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        float servoMaximumSpeed = 10f,
        float servoBaseSpeed = 1f,
        float servoMaximumForce = 1000f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_body.Changed(body) | reapplyInputs) _c.A = body;
        if (_targetOrientation.Changed(targetOrientation) | reapplyInputs) _c.TargetOrientation = targetOrientation;
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

/// <summary>Drives the velocity of a point on a single body towards a target.</summary>
[ProcessNode(Name = "OneBodyLinearMotor")]
public class OneBodyLinearMotorNode
{
    private readonly SConstraints.OneBodyLinearMotorConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _body;
    private PinValue<Vector3> _localOffset;
    private PinValue<Vector3> _targetVelocity;
    private PinValue<float> _motorDamping;
    private PinValue<float> _motorMaximumForce;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="body">The constrained body.</param>
    /// <param name="localOffset">Anchor point relative to the center of the body.</param>
    /// <param name="targetVelocity">Target velocity of the anchor point in world space, units per second.</param>
    /// <param name="motorDamping">How aggressively the motor corrects towards the target velocity.</param>
    /// <param name="motorMaximumForce">Maximum force the motor may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.OneBodyLinearMotorConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? body,
        Vector3 localOffset,
        Vector3 targetVelocity,
        float motorDamping = 10f,
        float motorMaximumForce = 1000f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_body.Changed(body) | reapplyInputs) _c.A = body;
        if (_localOffset.Changed(localOffset) | reapplyInputs) _c.LocalOffset = localOffset;
        if (_targetVelocity.Changed(targetVelocity) | reapplyInputs) _c.TargetVelocity = targetVelocity;
        if (_motorDamping.Changed(motorDamping) | reapplyInputs) _c.MotorDamping = motorDamping;
        if (_motorMaximumForce.Changed(motorMaximumForce) | reapplyInputs) _c.MotorMaximumForce = motorMaximumForce;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Servos a point on a single body towards a world-space target position.</summary>
[ProcessNode(Name = "OneBodyLinearServo")]
public class OneBodyLinearServoNode
{
    private readonly SConstraints.OneBodyLinearServoConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _body;
    private PinValue<Vector3> _localOffset;
    private PinValue<Vector3> _target;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<float> _servoMaximumSpeed;
    private PinValue<float> _servoBaseSpeed;
    private PinValue<float> _servoMaximumForce;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="body">The constrained body.</param>
    /// <param name="localOffset">Anchor point relative to the center of the body.</param>
    /// <param name="target">World-space position the servo drives the anchor point towards.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="servoMaximumSpeed">Maximum speed the servo may use to approach the target.</param>
    /// <param name="servoBaseSpeed">Minimum speed used while correcting remaining error.</param>
    /// <param name="servoMaximumForce">Maximum force the servo may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.OneBodyLinearServoConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? body,
        Vector3 localOffset,
        Vector3 target,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        float servoMaximumSpeed = 10f,
        float servoBaseSpeed = 1f,
        float servoMaximumForce = 1000f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_body.Changed(body) | reapplyInputs) _c.A = body;
        if (_localOffset.Changed(localOffset) | reapplyInputs) _c.LocalOffset = localOffset;
        if (_target.Changed(target) | reapplyInputs) _c.Target = target;
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
