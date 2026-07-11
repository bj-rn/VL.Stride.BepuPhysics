using VL.Core.Import;
using VL.Lib.Collections;
using SBepu = global::Stride.BepuPhysics;
using SConstraints = global::Stride.BepuPhysics.Constraints;

namespace VL.Stride.BepuPhysics.Constraints;

/// <summary>
/// Reads the constraints referencing a body, for example one obtained from a query or contact.
/// Use ConstraintInfo to identify each constraint, the capability operations (spring, motor,
/// servo) for the common tuning properties, or CastAs for type specific access.
/// </summary>
[ProcessNode(Name = "GetConstraints", Category = "Stride.Physics.Bepu.Constraints.Operations")]
public class GetConstraintsNode
{
    private readonly SpreadBuilder<SConstraints.ConstraintComponentBase> _builder = new();
    private Spread<SConstraints.ConstraintComponentBase> _constraints = Spread<SConstraints.ConstraintComponentBase>.Empty;

    /// <param name="body">The body whose constraints are read.</param>
    /// <param name="child">The constraint at Child Index. Null while the index is out of range.</param>
    /// <param name="childIndex">Index of the constraint to output on the Child pin.</param>
    /// <returns>All constraints referencing the body. Empty while none exist or no input.</returns>
    [return: Pin(Name = "Output")]
    public Spread<SConstraints.ConstraintComponentBase> Update(
        SBepu.BodyComponent? body,
        out SConstraints.ConstraintComponentBase? child,
        int childIndex = 0)
    {
        var list = body?.Constraints;
        if (!Matches(_constraints, list))
        {
            _builder.Clear();
            if (list is not null)
            {
                for (var i = 0; i < list.Count; i++)
                    _builder.Add(list[i]);
            }
            _constraints = _builder.ToSpread();
        }
        child = childIndex >= 0 && childIndex < _constraints.Count ? _constraints[childIndex] : null;
        return _constraints;
    }

    private static bool Matches(Spread<SConstraints.ConstraintComponentBase> spread, IReadOnlyList<SConstraints.ConstraintComponentBase>? list)
    {
        if (list is null)
            return spread.Count == 0;
        if (spread.Count != list.Count)
            return false;
        for (var i = 0; i < list.Count; i++)
        {
            if (!ReferenceEquals(spread[i], list[i]))
                return false;
        }
        return true;
    }
}

/// <summary>
/// Reads the bodies a constraint references, in slot order (A, B, C, D).
/// An entry is null while its slot is unassigned.
/// </summary>
[ProcessNode(Name = "GetConstraintBodies", Category = "Stride.Physics.Bepu.Constraints.Operations")]
public class GetConstraintBodiesNode
{
    private readonly SpreadBuilder<SBepu.BodyComponent?> _builder = new();
    private Spread<SBepu.BodyComponent?> _bodies = Spread<SBepu.BodyComponent?>.Empty;

    /// <param name="constraint">The constraint whose bodies are read, for example from a GetConstraints node.</param>
    /// <param name="child">The body at Child Index. Null while the index is out of range or the slot is unassigned.</param>
    /// <param name="childIndex">Index of the body to output on the Child pin (0 = A, 1 = B, ...).</param>
    /// <returns>The constraint's bodies in slot order (A, B, C, D). Empty while no input.</returns>
    [return: Pin(Name = "Output")]
    public Spread<SBepu.BodyComponent?> Update(
        SConstraints.ConstraintComponentBase? constraint,
        out SBepu.BodyComponent? child,
        int childIndex = 0)
    {
        var span = constraint is null
            ? ReadOnlySpan<SBepu.BodyComponent?>.Empty
            : constraint.Bodies;
        if (!Matches(_bodies, span))
        {
            _builder.Clear();
            foreach (var body in span)
                _builder.Add(body);
            _bodies = _builder.ToSpread();
        }
        child = childIndex >= 0 && childIndex < _bodies.Count ? _bodies[childIndex] : null;
        return _bodies;
    }

    private static bool Matches(Spread<SBepu.BodyComponent?> spread, ReadOnlySpan<SBepu.BodyComponent?> span)
    {
        if (spread.Count != span.Length)
            return false;
        for (var i = 0; i < span.Length; i++)
        {
            if (!ReferenceEquals(spread[i], span[i]))
                return false;
        }
        return true;
    }
}

/// <summary>
/// Operations on constraints obtained from a GetConstraints node: identification, applied
/// force readout, capability settings (spring, motor, servo, they work on every constraint
/// type that supports them and report whether it does) and type specific settings for all
/// 30 constraint types. Mutating operations run while Apply is true.
/// A written property is overwritten again once the owning constraint node's pin value changes.
/// </summary>
public static partial class Operations
{
    /// <summary>Identifies a constraint: its kind, state and which capability operations apply.</summary>
    /// <param name="constraint">The constraint to identify, for example from a GetConstraints node.</param>
    /// <param name="kind">Which constraint type this is. None while nothing is connected.</param>
    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="enabled">Whether the constraint is currently enabled.</param>
    /// <param name="isSpring">True when the spring operations apply (SpringSettings, SetSpringSettings).</param>
    /// <param name="isMotor">True when the motor operations apply (MotorSettings, SetMotorSettings).</param>
    /// <param name="isServo">True when the servo operations apply (ServoSettings, SetServoSettings).</param>
    public static void ConstraintInfo(SConstraints.ConstraintComponentBase? constraint,
        out ConstraintKind kind,
        out bool attached,
        out bool enabled,
        out bool isSpring,
        out bool isMotor,
        out bool isServo)
    {
        kind = constraint switch
        {
            SConstraints.BallSocketConstraintComponent => ConstraintKind.BallSocket,
            SConstraints.BallSocketMotorConstraintComponent => ConstraintKind.BallSocketMotor,
            SConstraints.BallSocketServoConstraintComponent => ConstraintKind.BallSocketServo,
            SConstraints.AngularHingeConstraintComponent => ConstraintKind.AngularHinge,
            SConstraints.AngularMotorConstraintComponent => ConstraintKind.AngularMotor,
            SConstraints.AngularServoConstraintComponent => ConstraintKind.AngularServo,
            SConstraints.AngularSwivelHingeConstraintComponent => ConstraintKind.AngularSwivelHinge,
            SConstraints.AngularAxisMotorConstraintComponent => ConstraintKind.AngularAxisMotor,
            SConstraints.AngularAxisGearMotorConstraintComponent => ConstraintKind.AngularAxisGearMotor,
            SConstraints.HingeConstraintComponent => ConstraintKind.Hinge,
            SConstraints.SwivelHingeConstraintComponent => ConstraintKind.SwivelHinge,
            SConstraints.SwingLimitConstraintComponent => ConstraintKind.SwingLimit,
            SConstraints.TwistLimitConstraintComponent => ConstraintKind.TwistLimit,
            SConstraints.TwistMotorConstraintComponent => ConstraintKind.TwistMotor,
            SConstraints.TwistServoConstraintComponent => ConstraintKind.TwistServo,
            SConstraints.WeldConstraintComponent => ConstraintKind.Weld,
            SConstraints.CenterDistanceConstraintComponent => ConstraintKind.CenterDistance,
            SConstraints.CenterDistanceLimitConstraintComponent => ConstraintKind.CenterDistanceLimit,
            SConstraints.DistanceLimitConstraintComponent => ConstraintKind.DistanceLimit,
            SConstraints.DistanceServoConstraintComponent => ConstraintKind.DistanceServo,
            SConstraints.LinearAxisLimitConstraintComponent => ConstraintKind.LinearAxisLimit,
            SConstraints.LinearAxisMotorConstraintComponent => ConstraintKind.LinearAxisMotor,
            SConstraints.LinearAxisServoConstraintComponent => ConstraintKind.LinearAxisServo,
            SConstraints.PointOnLineServoConstraintComponent => ConstraintKind.PointOnLineServo,
            SConstraints.OneBodyAngularMotorConstraintComponent => ConstraintKind.OneBodyAngularMotor,
            SConstraints.OneBodyAngularServoConstraintComponent => ConstraintKind.OneBodyAngularServo,
            SConstraints.OneBodyLinearMotorConstraintComponent => ConstraintKind.OneBodyLinearMotor,
            SConstraints.OneBodyLinearServoConstraintComponent => ConstraintKind.OneBodyLinearServo,
            SConstraints.AreaConstraintComponent => ConstraintKind.Area,
            SConstraints.VolumeConstraintComponent => ConstraintKind.Volume,
            _ => ConstraintKind.None,
        };
        attached = constraint?.Attached ?? false;
        enabled = constraint?.Enabled ?? false;
        isSpring = constraint is SConstraints.ISpring;
        isMotor = constraint is SConstraints.IMotor;
        isServo = constraint is SConstraints.IServo;
    }

    /// <summary>Reads how hard the constraint worked on the last physics tick.</summary>
    /// <param name="constraint">The constraint to read. Outputs zero while not attached.</param>
    /// <param name="forceSquared">Squared sum of all forces the constraint applied on the last tick. Compare with a motor's maximum force.</param>
    /// <param name="impulseSquared">Squared sum of all impulses the constraint applied on the last tick. Scales with the time step and sub steps, prefer Force Squared.</param>
    public static void ConstraintForce(SConstraints.ConstraintComponentBase? constraint,
        out float forceSquared,
        out float impulseSquared)
    {
        if (constraint is null || !constraint.Attached)
        {
            forceSquared = 0f;
            impulseSquared = 0f;
            return;
        }
        forceSquared = constraint.GetAccumulatedForceMagnitude();
        impulseSquared = constraint.GetAccumulatedImpulseMagnitude();
    }

    /// <summary>
    /// Enables or disables a constraint while Apply is true.
    /// The value is overwritten again once the owning constraint node's Enabled pin changes.
    /// </summary>
    /// <param name="constraint">The constraint to write to.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="apply">Writes the value each frame while true. Connect a Bang for a one-shot write.</param>
    [return: Pin(Name = "Output")]
    public static SConstraints.ConstraintComponentBase? SetConstraintEnabled(SConstraints.ConstraintComponentBase? constraint,
        bool enabled = true,
        bool apply = false)
    {
        if (apply && constraint is not null)
            constraint.Enabled = enabled;
        return constraint;
    }

    /// <summary>Reads the spring settings of any constraint that has them (see ConstraintInfo's Is Spring).</summary>
    /// <param name="constraint">The constraint to read. Outputs the defaults while null or without spring settings.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="hasSpring">True when the constraint has spring settings.</param>
    public static void SpringSettings(SConstraints.ConstraintComponentBase? constraint,
        out float springFrequency,
        out float springDampingRatio,
        out bool hasSpring)
    {
        if (constraint is SConstraints.ISpring spring)
        {
            springFrequency = spring.SpringFrequency;
            springDampingRatio = spring.SpringDampingRatio;
            hasSpring = true;
            return;
        }
        springFrequency = 30f;
        springDampingRatio = 5f;
        hasSpring = false;
    }

    /// <summary>
    /// Writes the spring settings of any constraint that has them while Apply is true.
    /// Does nothing on constraints without spring settings (see ConstraintInfo's Is Spring).
    /// A written property is overwritten again once the owning constraint node's pin value changes.
    /// </summary>
    /// <param name="constraint">The constraint to write to.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="apply">Writes all settings each frame while true. Connect a Bang for a one-shot write.</param>
    [return: Pin(Name = "Output")]
    public static SConstraints.ConstraintComponentBase? SetSpringSettings(SConstraints.ConstraintComponentBase? constraint,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool apply = false)
    {
        if (apply && constraint is SConstraints.ISpring spring)
        {
            spring.SpringFrequency = springFrequency;
            spring.SpringDampingRatio = springDampingRatio;
        }
        return constraint;
    }

    /// <summary>Reads the motor settings of any constraint that has them (see ConstraintInfo's Is Motor).</summary>
    /// <param name="constraint">The constraint to read. Outputs the defaults while null or without motor settings.</param>
    /// <param name="motorDamping">How aggressively the motor corrects towards the target velocity.</param>
    /// <param name="motorMaximumForce">Maximum force the motor may apply.</param>
    /// <param name="hasMotor">True when the constraint has motor settings.</param>
    public static void MotorSettings(SConstraints.ConstraintComponentBase? constraint,
        out float motorDamping,
        out float motorMaximumForce,
        out bool hasMotor)
    {
        if (constraint is SConstraints.IMotor motor)
        {
            motorDamping = motor.MotorDamping;
            motorMaximumForce = motor.MotorMaximumForce;
            hasMotor = true;
            return;
        }
        motorDamping = 10f;
        motorMaximumForce = 1000f;
        hasMotor = false;
    }

    /// <summary>
    /// Writes the motor settings of any constraint that has them while Apply is true.
    /// Does nothing on constraints without motor settings (see ConstraintInfo's Is Motor).
    /// A written property is overwritten again once the owning constraint node's pin value changes.
    /// </summary>
    /// <param name="constraint">The constraint to write to.</param>
    /// <param name="motorDamping">How aggressively the motor corrects towards the target velocity.</param>
    /// <param name="motorMaximumForce">Maximum force the motor may apply.</param>
    /// <param name="apply">Writes all settings each frame while true. Connect a Bang for a one-shot write.</param>
    [return: Pin(Name = "Output")]
    public static SConstraints.ConstraintComponentBase? SetMotorSettings(SConstraints.ConstraintComponentBase? constraint,
        float motorDamping = 10f,
        float motorMaximumForce = 1000f,
        bool apply = false)
    {
        if (apply && constraint is SConstraints.IMotor motor)
        {
            motor.MotorDamping = motorDamping;
            motor.MotorMaximumForce = motorMaximumForce;
        }
        return constraint;
    }

    /// <summary>Reads the servo settings of any constraint that has them (see ConstraintInfo's Is Servo).</summary>
    /// <param name="constraint">The constraint to read. Outputs the defaults while null or without servo settings.</param>
    /// <param name="servoMaximumSpeed">Maximum speed the servo may use to approach the target.</param>
    /// <param name="servoBaseSpeed">Minimum speed used while correcting remaining error.</param>
    /// <param name="servoMaximumForce">Maximum force the servo may apply.</param>
    /// <param name="hasServo">True when the constraint has servo settings.</param>
    public static void ServoSettings(SConstraints.ConstraintComponentBase? constraint,
        out float servoMaximumSpeed,
        out float servoBaseSpeed,
        out float servoMaximumForce,
        out bool hasServo)
    {
        if (constraint is SConstraints.IServo servo)
        {
            servoMaximumSpeed = servo.ServoMaximumSpeed;
            servoBaseSpeed = servo.ServoBaseSpeed;
            servoMaximumForce = servo.ServoMaximumForce;
            hasServo = true;
            return;
        }
        servoMaximumSpeed = 10f;
        servoBaseSpeed = 1f;
        servoMaximumForce = 1000f;
        hasServo = false;
    }

    /// <summary>
    /// Writes the servo settings of any constraint that has them while Apply is true.
    /// Does nothing on constraints without servo settings (see ConstraintInfo's Is Servo).
    /// A written property is overwritten again once the owning constraint node's pin value changes.
    /// </summary>
    /// <param name="constraint">The constraint to write to.</param>
    /// <param name="servoMaximumSpeed">Maximum speed the servo may use to approach the target.</param>
    /// <param name="servoBaseSpeed">Minimum speed used while correcting remaining error.</param>
    /// <param name="servoMaximumForce">Maximum force the servo may apply.</param>
    /// <param name="apply">Writes all settings each frame while true. Connect a Bang for a one-shot write.</param>
    [return: Pin(Name = "Output")]
    public static SConstraints.ConstraintComponentBase? SetServoSettings(SConstraints.ConstraintComponentBase? constraint,
        float servoMaximumSpeed = 10f,
        float servoBaseSpeed = 1f,
        float servoMaximumForce = 1000f,
        bool apply = false)
    {
        if (apply && constraint is SConstraints.IServo servo)
        {
            servo.ServoMaximumSpeed = servoMaximumSpeed;
            servo.ServoBaseSpeed = servoBaseSpeed;
            servo.ServoMaximumForce = servoMaximumForce;
        }
        return constraint;
    }
}
