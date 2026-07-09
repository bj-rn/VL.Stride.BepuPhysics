using System.ComponentModel;
using Stride.Core.Mathematics;
using VL.Core.Import;
using SBepu = global::Stride.BepuPhysics;
using SConstraints = global::Stride.BepuPhysics.Constraints;

namespace VL.Stride.BepuPhysics.Constraints;

/// <summary>Limits the offset between two anchors along an axis to a range.</summary>
[ProcessNode(Name = "LinearAxisLimit")]
public class LinearAxisLimitNode
{
    private readonly SConstraints.LinearAxisLimitConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffsetA">Anchor point relative to the center of body A.</param>
    /// <param name="localOffsetB">Anchor point relative to the center of body B.</param>
    /// <param name="localAxis">Sliding axis in the local space of body A.</param>
    /// <param name="minimumOffset">Smallest allowed offset between the anchors along the axis.</param>
    /// <param name="maximumOffset">Largest allowed offset between the anchors along the axis.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.LinearAxisLimitConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 localOffsetA,
        Vector3 localOffsetB,
        [DefaultValue("1.0, 1.0, 1.0")] Vector3 localAxis,
        float minimumOffset = 0f,
        float maximumOffset = 1f,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true)
    {
        if (localAxis == default) localAxis = Vector3.UnitY;
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalOffsetA != localOffsetA) _c.LocalOffsetA = localOffsetA;
        if (_c.LocalOffsetB != localOffsetB) _c.LocalOffsetB = localOffsetB;
        if (_c.LocalAxis != localAxis) _c.LocalAxis = localAxis;
        if (_c.MinimumOffset != minimumOffset) _c.MinimumOffset = minimumOffset;
        if (_c.MaximumOffset != maximumOffset) _c.MaximumOffset = maximumOffset;
        if (_c.SpringFrequency != springFrequency) _c.SpringFrequency = springFrequency;
        if (_c.SpringDampingRatio != springDampingRatio) _c.SpringDampingRatio = springDampingRatio;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Drives the offset between two anchors along an axis towards a target velocity.</summary>
[ProcessNode(Name = "LinearAxisMotor")]
public class LinearAxisMotorNode
{
    private readonly SConstraints.LinearAxisMotorConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffsetA">Anchor point relative to the center of body A.</param>
    /// <param name="localOffsetB">Anchor point relative to the center of body B.</param>
    /// <param name="localAxis">Sliding axis in the local space of body A.</param>
    /// <param name="targetVelocity">Target sliding velocity along the axis in units per second.</param>
    /// <param name="motorDamping">How aggressively the motor corrects towards the target velocity.</param>
    /// <param name="motorMaximumForce">Maximum force the motor may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.LinearAxisMotorConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 localOffsetA,
        Vector3 localOffsetB,
        [DefaultValue("1.0, 1.0, 1.0")] Vector3 localAxis,
        float targetVelocity = 0f,
        float motorDamping = 10f,
        float motorMaximumForce = 1000f,
        bool enabled = true)
    {
        if (localAxis == default) localAxis = Vector3.UnitY;
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalOffsetA != localOffsetA) _c.LocalOffsetA = localOffsetA;
        if (_c.LocalOffsetB != localOffsetB) _c.LocalOffsetB = localOffsetB;
        if (_c.LocalAxis != localAxis) _c.LocalAxis = localAxis;
        if (_c.TargetVelocity != targetVelocity) _c.TargetVelocity = targetVelocity;
        if (_c.MotorDamping != motorDamping) _c.MotorDamping = motorDamping;
        if (_c.MotorMaximumForce != motorMaximumForce) _c.MotorMaximumForce = motorMaximumForce;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Servos body B's anchor onto a plane defined on body A.</summary>
[ProcessNode(Name = "LinearAxisServo")]
public class LinearAxisServoNode
{
    private readonly SConstraints.LinearAxisServoConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffsetA">Anchor point relative to the center of body A.</param>
    /// <param name="localOffsetB">Anchor point relative to the center of body B.</param>
    /// <param name="localPlaneNormal">Plane normal in the local space of body A; the servo drives the anchor of B onto that plane.</param>
    /// <param name="targetOffset">Distance from the plane the servo drives towards.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="servoMaximumSpeed">Maximum speed the servo may use to approach the target.</param>
    /// <param name="servoBaseSpeed">Minimum speed used while correcting remaining error.</param>
    /// <param name="servoMaximumForce">Maximum force the servo may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.LinearAxisServoConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 localOffsetA,
        Vector3 localOffsetB,
        [DefaultValue("1.0, 1.0, 1.0")] Vector3 localPlaneNormal,
        float targetOffset = 0f,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        float servoMaximumSpeed = 10f,
        float servoBaseSpeed = 1f,
        float servoMaximumForce = 1000f,
        bool enabled = true)
    {
        if (localPlaneNormal == default) localPlaneNormal = Vector3.UnitY;
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalOffsetA != localOffsetA) _c.LocalOffsetA = localOffsetA;
        if (_c.LocalOffsetB != localOffsetB) _c.LocalOffsetB = localOffsetB;
        if (_c.LocalPlaneNormal != localPlaneNormal) _c.LocalPlaneNormal = localPlaneNormal;
        if (_c.TargetOffset != targetOffset) _c.TargetOffset = targetOffset;
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

/// <summary>Keeps body B's anchor on a line defined on body A.</summary>
[ProcessNode(Name = "PointOnLineServo")]
public class PointOnLineServoNode
{
    private readonly SConstraints.PointOnLineServoConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="localOffsetA">Anchor point relative to the center of body A.</param>
    /// <param name="localOffsetB">Anchor point relative to the center of body B.</param>
    /// <param name="localDirection">Line direction in the local space of body A; the anchor of B is kept on that line.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="servoMaximumSpeed">Maximum speed the servo may use to approach the target.</param>
    /// <param name="servoBaseSpeed">Minimum speed used while correcting remaining error.</param>
    /// <param name="servoMaximumForce">Maximum force the servo may apply.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.PointOnLineServoConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA,
        SBepu.BodyComponent? bodyB,
        Vector3 localOffsetA,
        Vector3 localOffsetB,
        [DefaultValue("1.0, 1.0, 1.0")] Vector3 localDirection,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        float servoMaximumSpeed = 10f,
        float servoBaseSpeed = 1f,
        float servoMaximumForce = 1000f,
        bool enabled = true)
    {
        if (localDirection == default) localDirection = Vector3.UnitY;
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalOffsetA != localOffsetA) _c.LocalOffsetA = localOffsetA;
        if (_c.LocalOffsetB != localOffsetB) _c.LocalOffsetB = localOffsetB;
        if (_c.LocalDirection != localDirection) _c.LocalDirection = localDirection;
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
