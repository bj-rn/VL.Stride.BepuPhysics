using System.ComponentModel;
using Stride.Core.Mathematics;
using VL.Core.Import;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
using SBepu = global::Stride.BepuPhysics;
using SConstraints = global::Stride.BepuPhysics.Constraints;

namespace VL.Stride.BepuPhysics.Constraints;

/// <summary>Limits the offset between two anchors along an axis to a range.</summary>
[ProcessNode(Name = "LinearAxisLimit")]
public class LinearAxisLimitNode
{
    private readonly SConstraints.LinearAxisLimitConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _localOffsetA;
    private PinValue<Vector3> _localOffsetB;
    private PinValue<Vector3> _localAxis;
    private PinValue<float> _minimumOffset;
    private PinValue<float> _maximumOffset;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffsetA">Anchor point relative to the center of body A.</param>
    /// <param name="localOffsetB">Anchor point relative to the center of body B.</param>
    /// <param name="localAxis">Sliding axis in the local space of body A. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="minimumOffset">Smallest allowed offset between the anchors along the axis.</param>
    /// <param name="maximumOffset">Largest allowed offset between the anchors along the axis.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.LinearAxisLimitConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 localOffsetA,
        Vector3 localOffsetB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localAxis,
        float minimumOffset = 0f,
        float maximumOffset = 1f,
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
        if (_localAxis.Changed(localAxis) | reapplyInputs) _c.LocalAxis = localAxis;
        if (_minimumOffset.Changed(minimumOffset) | reapplyInputs) _c.MinimumOffset = minimumOffset;
        if (_maximumOffset.Changed(maximumOffset) | reapplyInputs) _c.MaximumOffset = maximumOffset;
        if (_springFrequency.Changed(springFrequency) | reapplyInputs) _c.SpringFrequency = springFrequency;
        if (_springDampingRatio.Changed(springDampingRatio) | reapplyInputs) _c.SpringDampingRatio = springDampingRatio;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Drives the offset between two anchors along an axis towards a target velocity.</summary>
[ProcessNode(Name = "LinearAxisMotor")]
public class LinearAxisMotorNode
{
    private readonly SConstraints.LinearAxisMotorConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _localOffsetA;
    private PinValue<Vector3> _localOffsetB;
    private PinValue<Vector3> _localAxis;
    private PinValue<float> _targetVelocity;
    private PinValue<float> _motorDamping;
    private PinValue<float> _motorMaximumForce;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffsetA">Anchor point relative to the center of body A.</param>
    /// <param name="localOffsetB">Anchor point relative to the center of body B.</param>
    /// <param name="localAxis">Sliding axis in the local space of body A. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="targetVelocity">Target sliding velocity along the axis in units per second.</param>
    /// <param name="motorDamping">How aggressively the motor corrects towards the target velocity.</param>
    /// <param name="motorMaximumForce">Maximum force the motor may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.LinearAxisMotorConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 localOffsetA,
        Vector3 localOffsetB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localAxis,
        float targetVelocity = 0f,
        float motorDamping = 10f,
        float motorMaximumForce = 1000f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_localOffsetA.Changed(localOffsetA) | reapplyInputs) _c.LocalOffsetA = localOffsetA;
        if (_localOffsetB.Changed(localOffsetB) | reapplyInputs) _c.LocalOffsetB = localOffsetB;
        if (_localAxis.Changed(localAxis) | reapplyInputs) _c.LocalAxis = localAxis;
        if (_targetVelocity.Changed(targetVelocity) | reapplyInputs) _c.TargetVelocity = targetVelocity;
        if (_motorDamping.Changed(motorDamping) | reapplyInputs) _c.MotorDamping = motorDamping;
        if (_motorMaximumForce.Changed(motorMaximumForce) | reapplyInputs) _c.MotorMaximumForce = motorMaximumForce;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Servos body B's anchor onto a plane defined on body A.</summary>
[ProcessNode(Name = "LinearAxisServo")]
public class LinearAxisServoNode
{
    private readonly SConstraints.LinearAxisServoConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _localOffsetA;
    private PinValue<Vector3> _localOffsetB;
    private PinValue<Vector3> _localPlaneNormal;
    private PinValue<float> _targetOffset;
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
    /// <param name="localPlaneNormal">Plane normal in the local space of body A; the servo drives the anchor of B onto that plane. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="targetOffset">Distance from the plane the servo drives towards.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="servoMaximumSpeed">Maximum speed the servo may use to approach the target.</param>
    /// <param name="servoBaseSpeed">Minimum speed used while correcting remaining error.</param>
    /// <param name="servoMaximumForce">Maximum force the servo may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.LinearAxisServoConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 localOffsetA,
        Vector3 localOffsetB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localPlaneNormal,
        float targetOffset = 0f,
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
        if (_localPlaneNormal.Changed(localPlaneNormal) | reapplyInputs) _c.LocalPlaneNormal = localPlaneNormal;
        if (_targetOffset.Changed(targetOffset) | reapplyInputs) _c.TargetOffset = targetOffset;
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

/// <summary>Keeps body B's anchor on a line defined on body A.</summary>
[ProcessNode(Name = "PointOnLineServo")]
public class PointOnLineServoNode
{
    private readonly SConstraints.PointOnLineServoConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _localOffsetA;
    private PinValue<Vector3> _localOffsetB;
    private PinValue<Vector3> _localDirection;
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
    /// <param name="localDirection">Line direction in the local space of body A; the anchor of B is kept on that line. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="servoMaximumSpeed">Maximum speed the servo may use to approach the target.</param>
    /// <param name="servoBaseSpeed">Minimum speed used while correcting remaining error.</param>
    /// <param name="servoMaximumForce">Maximum force the servo may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.PointOnLineServoConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 localOffsetA,
        Vector3 localOffsetB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localDirection,
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
        if (_localDirection.Changed(localDirection) | reapplyInputs) _c.LocalDirection = localDirection;
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
