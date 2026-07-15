namespace VL.Stride.BepuPhysics.Components
{
    /// <summary>
    /// Continuous collision detection mode. Mirrors BepuPhysics.Collidables.ContinuousDetectionMode —
    /// defined here so the pin default resolves from a VL-imported assembly.
    /// </summary>
    public enum ContinuousDetectionKind
    {
        /// <summary>No sweep tests — fast objects may tunnel through thin geometry.</summary>
        Discrete = 0,
        /// <summary>No sweeps of its own, but other continuous collidables can see it.</summary>
        Passive = 1,
        /// <summary>Sweep tests prevent tunneling at higher cost.</summary>
        Continuous = 2,
    }

    /// <summary>What kind of collidable a CollidableComponent is.</summary>
    public enum CollidableKind
    {
        /// <summary>No collidable connected.</summary>
        None = 0,
        /// <summary>A dynamic or kinematic body (BodyComponent).</summary>
        Body = 1,
        /// <summary>Immovable collision geometry (StaticComponent).</summary>
        Static = 2,
    }
}

namespace VL.Stride.BepuPhysics.Colliders
{
    /// <summary>What shape a collider is.</summary>
    public enum ColliderKind
    {
        /// <summary>No collider connected.</summary>
        None = 0,
        /// <summary>A BoxCollider.</summary>
        Box = 1,
        /// <summary>A SphereCollider.</summary>
        Sphere = 2,
        /// <summary>A CapsuleCollider.</summary>
        Capsule = 3,
        /// <summary>A CylinderCollider.</summary>
        Cylinder = 4,
        /// <summary>A TriangleCollider.</summary>
        Triangle = 5,
        /// <summary>A ConvexHullCollider.</summary>
        ConvexHull = 6,
    }
}

namespace VL.Stride.BepuPhysics.Constraints
{
    /// <summary>What kind of constraint a ConstraintComponentBase is.</summary>
    public enum ConstraintKind
    {
        /// <summary>No constraint connected.</summary>
        None = 0,
        /// <summary>A BallSocket constraint.</summary>
        BallSocket,
        /// <summary>A BallSocketMotor constraint.</summary>
        BallSocketMotor,
        /// <summary>A BallSocketServo constraint.</summary>
        BallSocketServo,
        /// <summary>An AngularHinge constraint.</summary>
        AngularHinge,
        /// <summary>An AngularMotor constraint.</summary>
        AngularMotor,
        /// <summary>An AngularServo constraint.</summary>
        AngularServo,
        /// <summary>An AngularSwivelHinge constraint.</summary>
        AngularSwivelHinge,
        /// <summary>An AngularAxisMotor constraint.</summary>
        AngularAxisMotor,
        /// <summary>An AngularAxisGearMotor constraint.</summary>
        AngularAxisGearMotor,
        /// <summary>A Hinge constraint.</summary>
        Hinge,
        /// <summary>A SwivelHinge constraint.</summary>
        SwivelHinge,
        /// <summary>A SwingLimit constraint.</summary>
        SwingLimit,
        /// <summary>A TwistLimit constraint.</summary>
        TwistLimit,
        /// <summary>A TwistMotor constraint.</summary>
        TwistMotor,
        /// <summary>A TwistServo constraint.</summary>
        TwistServo,
        /// <summary>A Weld constraint.</summary>
        Weld,
        /// <summary>A CenterDistance constraint.</summary>
        CenterDistance,
        /// <summary>A CenterDistanceLimit constraint.</summary>
        CenterDistanceLimit,
        /// <summary>A DistanceLimit constraint.</summary>
        DistanceLimit,
        /// <summary>A DistanceServo constraint.</summary>
        DistanceServo,
        /// <summary>A LinearAxisLimit constraint.</summary>
        LinearAxisLimit,
        /// <summary>A LinearAxisMotor constraint.</summary>
        LinearAxisMotor,
        /// <summary>A LinearAxisServo constraint.</summary>
        LinearAxisServo,
        /// <summary>A PointOnLineServo constraint.</summary>
        PointOnLineServo,
        /// <summary>A OneBodyAngularMotor constraint.</summary>
        OneBodyAngularMotor,
        /// <summary>A OneBodyAngularServo constraint.</summary>
        OneBodyAngularServo,
        /// <summary>A OneBodyLinearMotor constraint.</summary>
        OneBodyLinearMotor,
        /// <summary>A OneBodyLinearServo constraint.</summary>
        OneBodyLinearServo,
        /// <summary>An Area constraint.</summary>
        Area,
        /// <summary>A Volume constraint.</summary>
        Volume,
    }
}
