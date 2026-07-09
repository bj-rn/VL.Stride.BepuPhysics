using System.ComponentModel;
using Stride.Core.Mathematics;
using VL.Core.Import;
using SBepu = global::Stride.BepuPhysics;
using SConstraints = global::Stride.BepuPhysics.Constraints;

namespace VL.Stride.BepuPhysics.Constraints;

/// <summary>Full hinge joint: anchors two bodies and allows rotation around a single axis.</summary>
[ProcessNode(Name = "Hinge")]
public class HingeNode
{
    private readonly SConstraints.HingeConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffsetA">Anchor point relative to the center of body A.</param>
    /// <param name="localHingeAxisA">Hinge axis in the local space of body A.</param>
    /// <param name="localOffsetB">Anchor point relative to the center of body B.</param>
    /// <param name="localHingeAxisB">Hinge axis in the local space of body B; kept aligned with the axis on A.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.HingeConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 localOffsetA,
        [DefaultValue("1.0, 1.0, 1.0")] Vector3 localHingeAxisA,
        Vector3 localOffsetB,
        [DefaultValue("1.0, 1.0, 1.0")] Vector3 localHingeAxisB,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true)
    {
        if (localHingeAxisA == default) localHingeAxisA = Vector3.UnitY;
        if (localHingeAxisB == default) localHingeAxisB = Vector3.UnitY;
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalOffsetA != localOffsetA) _c.LocalOffsetA = localOffsetA;
        if (_c.LocalHingeAxisA != localHingeAxisA) _c.LocalHingeAxisA = localHingeAxisA;
        if (_c.LocalOffsetB != localOffsetB) _c.LocalOffsetB = localOffsetB;
        if (_c.LocalHingeAxisB != localHingeAxisB) _c.LocalHingeAxisB = localHingeAxisB;
        if (_c.SpringFrequency != springFrequency) _c.SpringFrequency = springFrequency;
        if (_c.SpringDampingRatio != springDampingRatio) _c.SpringDampingRatio = springDampingRatio;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Anchors two bodies allowing rotation around a swivel axis on A and a hinge axis on B.</summary>
[ProcessNode(Name = "SwivelHinge")]
public class SwivelHingeNode
{
    private readonly SConstraints.SwivelHingeConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffsetA">Anchor point relative to the center of body A.</param>
    /// <param name="localSwivelAxisA">Free swivel axis in the local space of body A.</param>
    /// <param name="localOffsetB">Anchor point relative to the center of body B.</param>
    /// <param name="localHingeAxisB">Hinge axis in the local space of body B.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.SwivelHingeConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 localOffsetA,
        [DefaultValue("1.0, 1.0, 1.0")] Vector3 localSwivelAxisA,
        Vector3 localOffsetB,
        [DefaultValue("1.0, 1.0, 1.0")] Vector3 localHingeAxisB,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true)
    {
        if (localSwivelAxisA == default) localSwivelAxisA = Vector3.UnitY;
        if (localHingeAxisB == default) localHingeAxisB = Vector3.UnitY;
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalOffsetA != localOffsetA) _c.LocalOffsetA = localOffsetA;
        if (_c.LocalSwivelAxisA != localSwivelAxisA) _c.LocalSwivelAxisA = localSwivelAxisA;
        if (_c.LocalOffsetB != localOffsetB) _c.LocalOffsetB = localOffsetB;
        if (_c.LocalHingeAxisB != localHingeAxisB) _c.LocalHingeAxisB = localHingeAxisB;
        if (_c.SpringFrequency != springFrequency) _c.SpringFrequency = springFrequency;
        if (_c.SpringDampingRatio != springDampingRatio) _c.SpringDampingRatio = springDampingRatio;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Limits the swing angle between two axes on two bodies.</summary>
[ProcessNode(Name = "SwingLimit")]
public class SwingLimitNode
{
    private readonly SConstraints.SwingLimitConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="axisLocalA">Reference axis in the local space of body A.</param>
    /// <param name="axisLocalB">Measured axis in the local space of body B.</param>
    /// <param name="maximumSwingAngle">Largest allowed angle between the two axes in radians.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.SwingLimitConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        [DefaultValue("1.0, 1.0, 1.0")] Vector3 axisLocalA,
        [DefaultValue("1.0, 1.0, 1.0")] Vector3 axisLocalB,
        float maximumSwingAngle = 1f,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true)
    {
        if (axisLocalA == default) axisLocalA = Vector3.UnitY;
        if (axisLocalB == default) axisLocalB = Vector3.UnitY;
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.AxisLocalA != axisLocalA) _c.AxisLocalA = axisLocalA;
        if (_c.AxisLocalB != axisLocalB) _c.AxisLocalB = axisLocalB;
        if (_c.MaximumSwingAngle != maximumSwingAngle) _c.MaximumSwingAngle = maximumSwingAngle;
        if (_c.SpringFrequency != springFrequency) _c.SpringFrequency = springFrequency;
        if (_c.SpringDampingRatio != springDampingRatio) _c.SpringDampingRatio = springDampingRatio;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Limits the twist angle around the aligned basis axes of two bodies.</summary>
[ProcessNode(Name = "TwistLimit")]
public class TwistLimitNode
{
    private readonly SConstraints.TwistLimitConstraintComponent _c = new();

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
        bool enabled = true)
    {
        if (localBasisA == default) localBasisA = Quaternion.Identity;
        if (localBasisB == default) localBasisB = Quaternion.Identity;
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalBasisA != localBasisA) _c.LocalBasisA = localBasisA;
        if (_c.LocalBasisB != localBasisB) _c.LocalBasisB = localBasisB;
        if (_c.MinimumAngle != minimumAngle) _c.MinimumAngle = minimumAngle;
        if (_c.MaximumAngle != maximumAngle) _c.MaximumAngle = maximumAngle;
        if (_c.SpringFrequency != springFrequency) _c.SpringFrequency = springFrequency;
        if (_c.SpringDampingRatio != springDampingRatio) _c.SpringDampingRatio = springDampingRatio;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Drives the relative twist velocity around two axes towards a target.</summary>
[ProcessNode(Name = "TwistMotor")]
public class TwistMotorNode
{
    private readonly SConstraints.TwistMotorConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localAxisA">Twist axis in the local space of body A.</param>
    /// <param name="localAxisB">Twist axis in the local space of body B.</param>
    /// <param name="targetVelocity">Target twist velocity in radians per second.</param>
    /// <param name="motorDamping">How aggressively the motor corrects towards the target velocity.</param>
    /// <param name="motorMaximumForce">Maximum force the motor may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.TwistMotorConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        [DefaultValue("1.0, 1.0, 1.0")] Vector3 localAxisA,
        [DefaultValue("1.0, 1.0, 1.0")] Vector3 localAxisB,
        float targetVelocity = 0f,
        float motorDamping = 10f,
        float motorMaximumForce = 1000f,
        bool enabled = true)
    {
        if (localAxisA == default) localAxisA = Vector3.UnitY;
        if (localAxisB == default) localAxisB = Vector3.UnitY;
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalAxisA != localAxisA) _c.LocalAxisA = localAxisA;
        if (_c.LocalAxisB != localAxisB) _c.LocalAxisB = localAxisB;
        if (_c.TargetVelocity != targetVelocity) _c.TargetVelocity = targetVelocity;
        if (_c.MotorDamping != motorDamping) _c.MotorDamping = motorDamping;
        if (_c.MotorMaximumForce != motorMaximumForce) _c.MotorMaximumForce = motorMaximumForce;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Servos the twist angle around the aligned basis axes towards a target angle.</summary>
[ProcessNode(Name = "TwistServo")]
public class TwistServoNode
{
    private readonly SConstraints.TwistServoConstraintComponent _c = new();

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
        bool enabled = true)
    {
        if (localBasisA == default) localBasisA = Quaternion.Identity;
        if (localBasisB == default) localBasisB = Quaternion.Identity;
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalBasisA != localBasisA) _c.LocalBasisA = localBasisA;
        if (_c.LocalBasisB != localBasisB) _c.LocalBasisB = localBasisB;
        if (_c.TargetAngle != targetAngle) _c.TargetAngle = targetAngle;
        if (_c.SpringFrequency != springFrequency) _c.SpringFrequency = springFrequency;
        if (_c.SpringDampingRatio != springDampingRatio) _c.SpringDampingRatio = springDampingRatio;
        if (_c.ServoMaximumSpeed != servoMaximumSpeed) _c.ServoMaximumSpeed = servoMaximumSpeed;
        if (_c.ServoBaseSpeed != servoBaseSpeed) _c.ServoBaseSpeed = servoBaseSpeed;
        if (_c.ServoMaximumForce != servoMaximumForce) _c.ServoMaximumForce = servoMaximumForce;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Rigidly locks two bodies together (relative position and orientation).</summary>
[ProcessNode(Name = "Weld")]
public class WeldNode
{
    private readonly SConstraints.WeldConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffset">Position of body B relative to body A.</param>
    /// <param name="localOrientation">Orientation of body B relative to body A.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.WeldConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 localOffset,
        Quaternion localOrientation,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true)
    {
        if (localOrientation == default) localOrientation = Quaternion.Identity;
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalOffset != localOffset) _c.LocalOffset = localOffset;
        if (_c.LocalOrientation != localOrientation) _c.LocalOrientation = localOrientation;
        if (_c.SpringFrequency != springFrequency) _c.SpringFrequency = springFrequency;
        if (_c.SpringDampingRatio != springDampingRatio) _c.SpringDampingRatio = springDampingRatio;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}
