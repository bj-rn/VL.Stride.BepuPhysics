using System.ComponentModel;
using Stride.Core.Mathematics;
using VL.Core.Import;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
using SBepu = global::Stride.BepuPhysics;
using SConstraints = global::Stride.BepuPhysics.Constraints;

namespace VL.Stride.BepuPhysics.Constraints;

/// <summary>Full hinge joint: anchors two bodies and allows rotation around a single axis.</summary>
[ProcessNode(Name = "Hinge")]
public class HingeNode
{
    private readonly SConstraints.HingeConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _localOffsetA;
    private PinValue<Vector3> _localHingeAxisA;
    private PinValue<Vector3> _localOffsetB;
    private PinValue<Vector3> _localHingeAxisB;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffsetA">Anchor point relative to the center of body A.</param>
    /// <param name="localHingeAxisA">Hinge axis in the local space of body A. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="localOffsetB">Anchor point relative to the center of body B.</param>
    /// <param name="localHingeAxisB">Hinge axis in the local space of body B; kept aligned with the axis on A. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.HingeConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 localOffsetA,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localHingeAxisA,
        Vector3 localOffsetB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localHingeAxisB,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_localOffsetA.Changed(localOffsetA) | reapplyInputs) _c.LocalOffsetA = localOffsetA;
        if (_localHingeAxisA.Changed(localHingeAxisA) | reapplyInputs) _c.LocalHingeAxisA = localHingeAxisA;
        if (_localOffsetB.Changed(localOffsetB) | reapplyInputs) _c.LocalOffsetB = localOffsetB;
        if (_localHingeAxisB.Changed(localHingeAxisB) | reapplyInputs) _c.LocalHingeAxisB = localHingeAxisB;
        if (_springFrequency.Changed(springFrequency) | reapplyInputs) _c.SpringFrequency = springFrequency;
        if (_springDampingRatio.Changed(springDampingRatio) | reapplyInputs) _c.SpringDampingRatio = springDampingRatio;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Anchors two bodies allowing rotation around a swivel axis on A and a hinge axis on B.</summary>
[ProcessNode(Name = "SwivelHinge")]
public class SwivelHingeNode
{
    private readonly SConstraints.SwivelHingeConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _localOffsetA;
    private PinValue<Vector3> _localSwivelAxisA;
    private PinValue<Vector3> _localOffsetB;
    private PinValue<Vector3> _localHingeAxisB;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffsetA">Anchor point relative to the center of body A.</param>
    /// <param name="localSwivelAxisA">Free swivel axis in the local space of body A. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="localOffsetB">Anchor point relative to the center of body B.</param>
    /// <param name="localHingeAxisB">Hinge axis in the local space of body B. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.SwivelHingeConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 localOffsetA,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localSwivelAxisA,
        Vector3 localOffsetB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localHingeAxisB,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_localOffsetA.Changed(localOffsetA) | reapplyInputs) _c.LocalOffsetA = localOffsetA;
        if (_localSwivelAxisA.Changed(localSwivelAxisA) | reapplyInputs) _c.LocalSwivelAxisA = localSwivelAxisA;
        if (_localOffsetB.Changed(localOffsetB) | reapplyInputs) _c.LocalOffsetB = localOffsetB;
        if (_localHingeAxisB.Changed(localHingeAxisB) | reapplyInputs) _c.LocalHingeAxisB = localHingeAxisB;
        if (_springFrequency.Changed(springFrequency) | reapplyInputs) _c.SpringFrequency = springFrequency;
        if (_springDampingRatio.Changed(springDampingRatio) | reapplyInputs) _c.SpringDampingRatio = springDampingRatio;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Limits the swing angle between two axes on two bodies.</summary>
[ProcessNode(Name = "SwingLimit")]
public class SwingLimitNode
{
    private readonly SConstraints.SwingLimitConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _axisLocalA;
    private PinValue<Vector3> _axisLocalB;
    private PinValue<float> _maximumSwingAngle;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="axisLocalA">Reference axis in the local space of body A. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="axisLocalB">Measured axis in the local space of body B. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="maximumSwingAngle">Largest allowed angle between the two axes in radians.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.SwingLimitConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 axisLocalA,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 axisLocalB,
        float maximumSwingAngle = 1f,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_axisLocalA.Changed(axisLocalA) | reapplyInputs) _c.AxisLocalA = axisLocalA;
        if (_axisLocalB.Changed(axisLocalB) | reapplyInputs) _c.AxisLocalB = axisLocalB;
        if (_maximumSwingAngle.Changed(maximumSwingAngle) | reapplyInputs) _c.MaximumSwingAngle = maximumSwingAngle;
        if (_springFrequency.Changed(springFrequency) | reapplyInputs) _c.SpringFrequency = springFrequency;
        if (_springDampingRatio.Changed(springDampingRatio) | reapplyInputs) _c.SpringDampingRatio = springDampingRatio;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Limits the twist angle around the aligned basis axes of two bodies.</summary>
[ProcessNode(Name = "TwistLimit")]
public class TwistLimitNode
{
    private readonly SConstraints.TwistLimitConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Quaternion> _localBasisA;
    private PinValue<Quaternion> _localBasisB;
    private PinValue<float> _minimumAngle;
    private PinValue<float> _maximumAngle;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localBasisA">Twist basis in the local space of body A (twist measured around its Y axis).</param>
    /// <param name="localBasisB">Twist basis in the local space of body B.</param>
    /// <param name="minimumAngle">Smallest allowed twist angle in radians.</param>
    /// <param name="maximumAngle">Largest allowed twist angle in radians.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.TwistLimitConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Quaternion localBasisA,
        Quaternion localBasisB,
        float minimumAngle = -1f,
        float maximumAngle = 1f,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_localBasisA.Changed(localBasisA) | reapplyInputs) _c.LocalBasisA = localBasisA;
        if (_localBasisB.Changed(localBasisB) | reapplyInputs) _c.LocalBasisB = localBasisB;
        if (_minimumAngle.Changed(minimumAngle) | reapplyInputs) _c.MinimumAngle = minimumAngle;
        if (_maximumAngle.Changed(maximumAngle) | reapplyInputs) _c.MaximumAngle = maximumAngle;
        if (_springFrequency.Changed(springFrequency) | reapplyInputs) _c.SpringFrequency = springFrequency;
        if (_springDampingRatio.Changed(springDampingRatio) | reapplyInputs) _c.SpringDampingRatio = springDampingRatio;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Drives the relative twist velocity around two axes towards a target.</summary>
[ProcessNode(Name = "TwistMotor")]
public class TwistMotorNode
{
    private readonly SConstraints.TwistMotorConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _localAxisA;
    private PinValue<Vector3> _localAxisB;
    private PinValue<float> _targetVelocity;
    private PinValue<float> _motorDamping;
    private PinValue<float> _motorMaximumForce;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localAxisA">Twist axis in the local space of body A. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="localAxisB">Twist axis in the local space of body B. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="targetVelocity">Target twist velocity in radians per second.</param>
    /// <param name="motorDamping">How aggressively the motor corrects towards the target velocity.</param>
    /// <param name="motorMaximumForce">Maximum force the motor may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.TwistMotorConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localAxisA,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localAxisB,
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
        if (_localAxisB.Changed(localAxisB) | reapplyInputs) _c.LocalAxisB = localAxisB;
        if (_targetVelocity.Changed(targetVelocity) | reapplyInputs) _c.TargetVelocity = targetVelocity;
        if (_motorDamping.Changed(motorDamping) | reapplyInputs) _c.MotorDamping = motorDamping;
        if (_motorMaximumForce.Changed(motorMaximumForce) | reapplyInputs) _c.MotorMaximumForce = motorMaximumForce;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Servos the twist angle around the aligned basis axes towards a target angle.</summary>
[ProcessNode(Name = "TwistServo")]
public class TwistServoNode
{
    private readonly SConstraints.TwistServoConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Quaternion> _localBasisA;
    private PinValue<Quaternion> _localBasisB;
    private PinValue<float> _targetAngle;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<float> _servoMaximumSpeed;
    private PinValue<float> _servoBaseSpeed;
    private PinValue<float> _servoMaximumForce;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localBasisA">Twist basis in the local space of body A (twist measured around its Y axis).</param>
    /// <param name="localBasisB">Twist basis in the local space of body B.</param>
    /// <param name="targetAngle">Twist angle in radians the servo drives towards.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="servoMaximumSpeed">Maximum speed the servo may use to approach the target.</param>
    /// <param name="servoBaseSpeed">Minimum speed used while correcting remaining error.</param>
    /// <param name="servoMaximumForce">Maximum force the servo may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.TwistServoConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Quaternion localBasisA,
        Quaternion localBasisB,
        float targetAngle = 0f,
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
        if (_localBasisA.Changed(localBasisA) | reapplyInputs) _c.LocalBasisA = localBasisA;
        if (_localBasisB.Changed(localBasisB) | reapplyInputs) _c.LocalBasisB = localBasisB;
        if (_targetAngle.Changed(targetAngle) | reapplyInputs) _c.TargetAngle = targetAngle;
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

/// <summary>Rigidly locks two bodies together (relative position and orientation).</summary>
[ProcessNode(Name = "Weld")]
public class WeldNode
{
    private readonly SConstraints.WeldConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<Vector3> _localOffset;
    private PinValue<Quaternion> _localOrientation;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffset">Position of body B relative to body A.</param>
    /// <param name="localOrientation">Orientation of body B relative to body A.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.WeldConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 localOffset,
        Quaternion localOrientation,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_localOffset.Changed(localOffset) | reapplyInputs) _c.LocalOffset = localOffset;
        if (_localOrientation.Changed(localOrientation) | reapplyInputs) _c.LocalOrientation = localOrientation;
        if (_springFrequency.Changed(springFrequency) | reapplyInputs) _c.SpringFrequency = springFrequency;
        if (_springDampingRatio.Changed(springDampingRatio) | reapplyInputs) _c.SpringDampingRatio = springDampingRatio;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}
