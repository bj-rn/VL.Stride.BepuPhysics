using Stride.Core.Mathematics;
using VL.Core.Import;
using SBepu = global::Stride.BepuPhysics;
using SConstraints = global::Stride.BepuPhysics.Constraints;

namespace VL.Stride.BepuPhysics.Constraints;

/// <summary>Ball and socket joint: keeps two anchor points on two bodies together.</summary>
[ProcessNode(Name = "BallSocket")]
public class BallSocketNode
{
    private readonly SConstraints.BallSocketConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffsetA">Anchor point relative to the center of body A.</param>
    /// <param name="localOffsetB">Anchor point relative to the center of body B.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.BallSocketConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA = null,
        SBepu.BodyComponent? bodyB = null,
        Vector3 localOffsetA = default,
        Vector3 localOffsetB = default,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true)
    {
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalOffsetA != localOffsetA) _c.LocalOffsetA = localOffsetA;
        if (_c.LocalOffsetB != localOffsetB) _c.LocalOffsetB = localOffsetB;
        if (_c.SpringFrequency != springFrequency) _c.SpringFrequency = springFrequency;
        if (_c.SpringDampingRatio != springDampingRatio) _c.SpringDampingRatio = springDampingRatio;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Ball socket joint that drives a target velocity at the anchor point.</summary>
[ProcessNode(Name = "BallSocketMotor")]
public class BallSocketMotorNode
{
    private readonly SConstraints.BallSocketMotorConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffsetB">Anchor point relative to the center of body B.</param>
    /// <param name="targetVelocityLocalA">Target velocity of the anchor on body B, expressed in the local space of body A.</param>
    /// <param name="motorDamping">How aggressively the motor corrects towards the target velocity.</param>
    /// <param name="motorMaximumForce">Maximum force the motor may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.BallSocketMotorConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA = null,
        SBepu.BodyComponent? bodyB = null,
        Vector3 localOffsetB = default,
        Vector3 targetVelocityLocalA = default,
        float motorDamping = 10f,
        float motorMaximumForce = 1000f,
        bool enabled = true)
    {
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalOffsetB != localOffsetB) _c.LocalOffsetB = localOffsetB;
        if (_c.TargetVelocityLocalA != targetVelocityLocalA) _c.TargetVelocityLocalA = targetVelocityLocalA;
        if (_c.MotorDamping != motorDamping) _c.MotorDamping = motorDamping;
        if (_c.MotorMaximumForce != motorMaximumForce) _c.MotorMaximumForce = motorMaximumForce;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Ball socket joint that servos the anchor of body B towards the anchor of body A.</summary>
[ProcessNode(Name = "BallSocketServo")]
public class BallSocketServoNode
{
    private readonly SConstraints.BallSocketServoConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffsetA">Anchor point relative to the center of body A.</param>
    /// <param name="localOffsetB">Anchor point relative to the center of body B.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="servoMaximumSpeed">Maximum speed the servo may use to approach the target.</param>
    /// <param name="servoBaseSpeed">Minimum speed used while correcting remaining error.</param>
    /// <param name="servoMaximumForce">Maximum force the servo may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.BallSocketServoConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA = null,
        SBepu.BodyComponent? bodyB = null,
        Vector3 localOffsetA = default,
        Vector3 localOffsetB = default,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        float servoMaximumSpeed = 10f,
        float servoBaseSpeed = 1f,
        float servoMaximumForce = 1000f,
        bool enabled = true)
    {
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalOffsetA != localOffsetA) _c.LocalOffsetA = localOffsetA;
        if (_c.LocalOffsetB != localOffsetB) _c.LocalOffsetB = localOffsetB;
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
