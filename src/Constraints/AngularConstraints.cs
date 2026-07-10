using System.ComponentModel;
using Stride.Core.Mathematics;
using VL.Core.Import;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
using SBepu = global::Stride.BepuPhysics;
using SConstraints = global::Stride.BepuPhysics.Constraints;

namespace VL.Stride.BepuPhysics.Constraints;

/// <summary>Aligns two local axes on two bodies, like a hinge without position lock.</summary>
[ProcessNode(Name = "AngularHinge")]
public class AngularHingeNode
{
    private readonly SConstraints.AngularHingeConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _localHingeAxisA;
    private PinValue<Vector3> _localHingeAxisB;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localHingeAxisA">Hinge axis in the local space of body A. Must be a non-zero (unit-length) vector, a zero axis produces NaN poses in the solver.</param>
    /// <param name="localHingeAxisB">Hinge axis in the local space of body B; kept aligned with the axis on A. Must be a non-zero (unit-length) vector, a zero axis produces NaN poses in the solver.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.AngularHingeConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localHingeAxisA,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localHingeAxisB,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_localHingeAxisA.Changed(localHingeAxisA) | reapplyInputs) _c.LocalHingeAxisA = localHingeAxisA;
        if (_localHingeAxisB.Changed(localHingeAxisB) | reapplyInputs) _c.LocalHingeAxisB = localHingeAxisB;
        if (_springFrequency.Changed(springFrequency) | reapplyInputs) _c.SpringFrequency = springFrequency;
        if (_springDampingRatio.Changed(springDampingRatio) | reapplyInputs) _c.SpringDampingRatio = springDampingRatio;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Drives the relative angular velocity between two bodies towards a target.</summary>
[ProcessNode(Name = "AngularMotor")]
public class AngularMotorNode
{
    private readonly SConstraints.AngularMotorConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _targetVelocityLocalA;
    private PinValue<float> _motorDamping;
    private PinValue<float> _motorMaximumForce;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="targetVelocityLocalA">Target relative angular velocity, expressed in the local space of body A.</param>
    /// <param name="motorDamping">How aggressively the motor corrects towards the target velocity.</param>
    /// <param name="motorMaximumForce">Maximum force the motor may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.AngularMotorConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 targetVelocityLocalA,
        float motorDamping = 10f,
        float motorMaximumForce = 1000f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_targetVelocityLocalA.Changed(targetVelocityLocalA) | reapplyInputs) _c.TargetVelocityLocalA = targetVelocityLocalA;
        if (_motorDamping.Changed(motorDamping) | reapplyInputs) _c.MotorDamping = motorDamping;
        if (_motorMaximumForce.Changed(motorMaximumForce) | reapplyInputs) _c.MotorMaximumForce = motorMaximumForce;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Servos the relative orientation between two bodies towards a target rotation.</summary>
[ProcessNode(Name = "AngularServo")]
public class AngularServoNode
{
    private readonly SConstraints.AngularServoConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Quaternion> _targetRelativeRotationLocalA;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<float> _servoMaximumSpeed;
    private PinValue<float> _servoBaseSpeed;
    private PinValue<float> _servoMaximumForce;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="targetRelativeRotationLocalA">Target orientation of body B relative to body A, in the local space of A.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="servoMaximumSpeed">Maximum speed the servo may use to approach the target.</param>
    /// <param name="servoBaseSpeed">Minimum speed used while correcting remaining error.</param>
    /// <param name="servoMaximumForce">Maximum force the servo may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.AngularServoConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Quaternion targetRelativeRotationLocalA,
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
        if (_targetRelativeRotationLocalA.Changed(targetRelativeRotationLocalA) | reapplyInputs) _c.TargetRelativeRotationLocalA = targetRelativeRotationLocalA;
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

/// <summary>Like AngularHinge but with a free swivel axis (universal-joint-like).</summary>
[ProcessNode(Name = "AngularSwivelHinge")]
public class AngularSwivelHingeNode
{
    private readonly SConstraints.AngularSwivelHingeConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _localSwivelAxisA;
    private PinValue<Vector3> _localHingeAxisB;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localSwivelAxisA">Free swivel axis in the local space of body A. Must be a non-zero (unit-length) vector, a zero axis produces NaN poses in the solver.</param>
    /// <param name="localHingeAxisB">Hinge axis in the local space of body B; kept perpendicular to the swivel axis. Must be a non-zero (unit-length) vector, a zero axis produces NaN poses in the solver.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.AngularSwivelHingeConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localSwivelAxisA,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localHingeAxisB,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_localSwivelAxisA.Changed(localSwivelAxisA) | reapplyInputs) _c.LocalSwivelAxisA = localSwivelAxisA;
        if (_localHingeAxisB.Changed(localHingeAxisB) | reapplyInputs) _c.LocalHingeAxisB = localHingeAxisB;
        if (_springFrequency.Changed(springFrequency) | reapplyInputs) _c.SpringFrequency = springFrequency;
        if (_springDampingRatio.Changed(springDampingRatio) | reapplyInputs) _c.SpringDampingRatio = springDampingRatio;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Drives rotation around an axis of body A towards a target velocity.</summary>
[ProcessNode(Name = "AngularAxisMotor")]
public class AngularAxisMotorNode
{
    private readonly SConstraints.AngularAxisMotorConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _localAxisA;
    private PinValue<float> _targetVelocity;
    private PinValue<float> _motorDamping;
    private PinValue<float> _motorMaximumForce;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localAxisA">Rotation axis in the local space of body A. Must be a non-zero (unit-length) vector, a zero axis produces NaN poses in the solver.</param>
    /// <param name="targetVelocity">Target angular velocity around the axis in radians per second.</param>
    /// <param name="motorDamping">How aggressively the motor corrects towards the target velocity.</param>
    /// <param name="motorMaximumForce">Maximum force the motor may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.AngularAxisMotorConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localAxisA,
        float targetVelocity = 0f,
        float motorDamping = 10f,
        float motorMaximumForce = 1000f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_localAxisA.Changed(localAxisA) | reapplyInputs) _c.LocalAxisA = localAxisA;
        if (_targetVelocity.Changed(targetVelocity) | reapplyInputs) _c.TargetVelocity = targetVelocity;
        if (_motorDamping.Changed(motorDamping) | reapplyInputs) _c.MotorDamping = motorDamping;
        if (_motorMaximumForce.Changed(motorMaximumForce) | reapplyInputs) _c.MotorMaximumForce = motorMaximumForce;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Couples rotation of two bodies around an axis with a gear-like velocity ratio.</summary>
[ProcessNode(Name = "AngularAxisGearMotor")]
public class AngularAxisGearMotorNode
{
    private readonly SConstraints.AngularAxisGearMotorConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _localAxisA;
    private PinValue<float> _velocityScale;
    private PinValue<float> _motorDamping;
    private PinValue<float> _motorMaximumForce;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localAxisA">Rotation axis in the local space of body A. Must be a non-zero (unit-length) vector, a zero axis produces NaN poses in the solver.</param>
    /// <param name="velocityScale">Gear ratio: angular velocity of body B relative to body A around the axis.</param>
    /// <param name="motorDamping">How aggressively the motor corrects towards the target velocity.</param>
    /// <param name="motorMaximumForce">Maximum force the motor may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.AngularAxisGearMotorConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localAxisA,
        float velocityScale = 1f,
        float motorDamping = 10f,
        float motorMaximumForce = 1000f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_localAxisA.Changed(localAxisA) | reapplyInputs) _c.LocalAxisA = localAxisA;
        if (_velocityScale.Changed(velocityScale) | reapplyInputs) _c.VelocityScale = velocityScale;
        if (_motorDamping.Changed(motorDamping) | reapplyInputs) _c.MotorDamping = motorDamping;
        if (_motorMaximumForce.Changed(motorMaximumForce) | reapplyInputs) _c.MotorMaximumForce = motorMaximumForce;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}
