namespace VL.Stride.BepuPhysics;

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
