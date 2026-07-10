using System.ComponentModel;
using Stride.Core.Mathematics;
using VL.Core.Import;
using SBepu = global::Stride.BepuPhysics;
using SConstraints = global::Stride.BepuPhysics.Constraints;

namespace VL.Stride.BepuPhysics.Constraints;

/// <summary>Aligns two local axes on two bodies, like a hinge without position lock.</summary>
[ProcessNode(Name = "AngularHinge")]
public class AngularHingeNode
{
    private readonly SConstraints.AngularHingeConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localHingeAxisA">Hinge axis in the local space of body A. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="localHingeAxisB">Hinge axis in the local space of body B; kept aligned with the axis on A. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.AngularHingeConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localHingeAxisA,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localHingeAxisB,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true)
    {
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalHingeAxisA != localHingeAxisA) _c.LocalHingeAxisA = localHingeAxisA;
        if (_c.LocalHingeAxisB != localHingeAxisB) _c.LocalHingeAxisB = localHingeAxisB;
        if (_c.SpringFrequency != springFrequency) _c.SpringFrequency = springFrequency;
        if (_c.SpringDampingRatio != springDampingRatio) _c.SpringDampingRatio = springDampingRatio;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Drives the relative angular velocity between two bodies towards a target.</summary>
[ProcessNode(Name = "AngularMotor")]
public class AngularMotorNode
{
    private readonly SConstraints.AngularMotorConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="targetVelocityLocalA">Target relative angular velocity, expressed in the local space of body A.</param>
    /// <param name="motorDamping">How aggressively the motor corrects towards the target velocity.</param>
    /// <param name="motorMaximumForce">Maximum force the motor may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.AngularMotorConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 targetVelocityLocalA,
        float motorDamping = 10f,
        float motorMaximumForce = 1000f,
        bool enabled = true)
    {
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.TargetVelocityLocalA != targetVelocityLocalA) _c.TargetVelocityLocalA = targetVelocityLocalA;
        if (_c.MotorDamping != motorDamping) _c.MotorDamping = motorDamping;
        if (_c.MotorMaximumForce != motorMaximumForce) _c.MotorMaximumForce = motorMaximumForce;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Servos the relative orientation between two bodies towards a target rotation.</summary>
[ProcessNode(Name = "AngularServo")]
public class AngularServoNode
{
    private readonly SConstraints.AngularServoConstraintComponent _c = new();

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
        bool enabled = true)
    {
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.TargetRelativeRotationLocalA != targetRelativeRotationLocalA) _c.TargetRelativeRotationLocalA = targetRelativeRotationLocalA;
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

/// <summary>Like AngularHinge but with a free swivel axis (universal-joint-like).</summary>
[ProcessNode(Name = "AngularSwivelHinge")]
public class AngularSwivelHingeNode
{
    private readonly SConstraints.AngularSwivelHingeConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localSwivelAxisA">Free swivel axis in the local space of body A. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="localHingeAxisB">Hinge axis in the local space of body B; kept perpendicular to the swivel axis. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.AngularSwivelHingeConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localSwivelAxisA,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localHingeAxisB,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true)
    {
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalSwivelAxisA != localSwivelAxisA) _c.LocalSwivelAxisA = localSwivelAxisA;
        if (_c.LocalHingeAxisB != localHingeAxisB) _c.LocalHingeAxisB = localHingeAxisB;
        if (_c.SpringFrequency != springFrequency) _c.SpringFrequency = springFrequency;
        if (_c.SpringDampingRatio != springDampingRatio) _c.SpringDampingRatio = springDampingRatio;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Drives rotation around an axis of body A towards a target velocity.</summary>
[ProcessNode(Name = "AngularAxisMotor")]
public class AngularAxisMotorNode
{
    private readonly SConstraints.AngularAxisMotorConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localAxisA">Rotation axis in the local space of body A. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="targetVelocity">Target angular velocity around the axis in radians per second.</param>
    /// <param name="motorDamping">How aggressively the motor corrects towards the target velocity.</param>
    /// <param name="motorMaximumForce">Maximum force the motor may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.AngularAxisMotorConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localAxisA,
        float targetVelocity = 0f,
        float motorDamping = 10f,
        float motorMaximumForce = 1000f,
        bool enabled = true)
    {
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalAxisA != localAxisA) _c.LocalAxisA = localAxisA;
        if (_c.TargetVelocity != targetVelocity) _c.TargetVelocity = targetVelocity;
        if (_c.MotorDamping != motorDamping) _c.MotorDamping = motorDamping;
        if (_c.MotorMaximumForce != motorMaximumForce) _c.MotorMaximumForce = motorMaximumForce;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Couples rotation of two bodies around an axis with a gear-like velocity ratio.</summary>
[ProcessNode(Name = "AngularAxisGearMotor")]
public class AngularAxisGearMotorNode
{
    private readonly SConstraints.AngularAxisGearMotorConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localAxisA">Rotation axis in the local space of body A. Must be a non-zero (unit-length) vector ,a zero axis produces NaN poses in the solver.</param>
    /// <param name="velocityScale">Gear ratio: angular velocity of body B relative to body A around the axis.</param>
    /// <param name="motorDamping">How aggressively the motor corrects towards the target velocity.</param>
    /// <param name="motorMaximumForce">Maximum force the motor may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.AngularAxisGearMotorConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        [DefaultValue("0.0, 1.0, 0.0")] Vector3 localAxisA,
        float velocityScale = 1f,
        float motorDamping = 10f,
        float motorMaximumForce = 1000f,
        bool enabled = true)
    {
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalAxisA != localAxisA) _c.LocalAxisA = localAxisA;
        if (_c.VelocityScale != velocityScale) _c.VelocityScale = velocityScale;
        if (_c.MotorDamping != motorDamping) _c.MotorDamping = motorDamping;
        if (_c.MotorMaximumForce != motorMaximumForce) _c.MotorMaximumForce = motorMaximumForce;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}
