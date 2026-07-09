using Stride.Core.Mathematics;
using VL.Core.Import;
using SBepu = global::Stride.BepuPhysics;
using SConstraints = global::Stride.BepuPhysics.Constraints;

namespace VL.Stride.BepuPhysics.Constraints;

/// <summary>Keeps the centers of two bodies at a target distance.</summary>
[ProcessNode(Name = "CenterDistance")]
public class CenterDistanceNode
{
    private readonly SConstraints.CenterDistanceConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="targetDistance">Distance to maintain between the two body centers.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.CenterDistanceConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA = null,
        SBepu.BodyComponent? bodyB = null,
        float targetDistance = 1f,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true)
    {
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.TargetDistance != targetDistance) _c.TargetDistance = targetDistance;
        if (_c.SpringFrequency != springFrequency) _c.SpringFrequency = springFrequency;
        if (_c.SpringDampingRatio != springDampingRatio) _c.SpringDampingRatio = springDampingRatio;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Keeps the distance between the centers of two bodies within a range.</summary>
[ProcessNode(Name = "CenterDistanceLimit")]
public class CenterDistanceLimitNode
{
    private readonly SConstraints.CenterDistanceLimitConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="minimumDistance">Smallest allowed distance between the two body centers.</param>
    /// <param name="maximumDistance">Largest allowed distance between the two body centers.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.CenterDistanceLimitConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA = null,
        SBepu.BodyComponent? bodyB = null,
        float minimumDistance = 0f,
        float maximumDistance = 1f,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true)
    {
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.MinimumDistance != minimumDistance) _c.MinimumDistance = minimumDistance;
        if (_c.MaximumDistance != maximumDistance) _c.MaximumDistance = maximumDistance;
        if (_c.SpringFrequency != springFrequency) _c.SpringFrequency = springFrequency;
        if (_c.SpringDampingRatio != springDampingRatio) _c.SpringDampingRatio = springDampingRatio;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Keeps the distance between two anchor points within a range.</summary>
[ProcessNode(Name = "DistanceLimit")]
public class DistanceLimitNode
{
    private readonly SConstraints.DistanceLimitConstraintComponent _c = new();

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
        bool enabled = true)
    {
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalOffsetA != localOffsetA) _c.LocalOffsetA = localOffsetA;
        if (_c.LocalOffsetB != localOffsetB) _c.LocalOffsetB = localOffsetB;
        if (_c.MinimumDistance != minimumDistance) _c.MinimumDistance = minimumDistance;
        if (_c.MaximumDistance != maximumDistance) _c.MaximumDistance = maximumDistance;
        if (_c.SpringFrequency != springFrequency) _c.SpringFrequency = springFrequency;
        if (_c.SpringDampingRatio != springDampingRatio) _c.SpringDampingRatio = springDampingRatio;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Servos the distance between two anchor points towards a target.</summary>
[ProcessNode(Name = "DistanceServo")]
public class DistanceServoNode
{
    private readonly SConstraints.DistanceServoConstraintComponent _c = new();

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
        bool enabled = true)
    {
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (_c.LocalOffsetA != localOffsetA) _c.LocalOffsetA = localOffsetA;
        if (_c.LocalOffsetB != localOffsetB) _c.LocalOffsetB = localOffsetB;
        if (_c.TargetDistance != targetDistance) _c.TargetDistance = targetDistance;
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
