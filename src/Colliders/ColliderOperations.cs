using Stride.Core.Mathematics;
using SColliders = global::Stride.BepuPhysics.Definitions.Colliders;

namespace VL.Stride.BepuPhysics.Colliders;

/// <summary>
/// Read access to collider shapes, for example from a GetColliders node.
/// Use CastAs (BoxCollider) to narrow a ColliderBase for the shape specific getters.
/// </summary>
public static class ColliderOperations
{
    /// <summary>Identifies which shape a collider is, for routing before CastAs.</summary>
    /// <param name="collider">The collider shape to identify.</param>
    /// <param name="kind">Which shape the collider is. None while nothing is connected.</param>
    public static void GetColliderInfo(SColliders.ColliderBase? collider,
        out ColliderKind kind)
    {
        kind = collider switch
        {
            SColliders.BoxCollider => ColliderKind.Box,
            SColliders.SphereCollider => ColliderKind.Sphere,
            SColliders.CapsuleCollider => ColliderKind.Capsule,
            SColliders.CylinderCollider => ColliderKind.Cylinder,
            SColliders.TriangleCollider => ColliderKind.Triangle,
            SColliders.ConvexHullCollider => ColliderKind.ConvexHull,
            _ => ColliderKind.None,
        };
    }

    /// <summary>Reads the placement properties shared by all collider shapes.</summary>
    /// <param name="collider">The collider shape to read. Outputs the defaults while null.</param>
    /// <param name="positionLocal">Position of the shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of the shape relative to the body.</param>
    /// <param name="mass">Relative weight of the shape within its compound.</param>
    public static void GetColliderSettings(SColliders.ColliderBase? collider,
        out Vector3 positionLocal,
        out Quaternion rotationLocal,
        out float mass)
    {
        positionLocal = collider?.PositionLocal ?? Vector3.Zero;
        rotationLocal = collider?.RotationLocal ?? Quaternion.Identity;
        mass = collider?.Mass ?? 1f;
    }

    /// <summary>Reads all settings of a box collider, mirroring the BoxCollider node's inputs.</summary>
    /// <param name="collider">The box collider to read. Outputs the defaults while null.</param>
    /// <param name="size">Extents of the box in meters.</param>
    /// <param name="positionLocal">Position of the shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of the shape relative to the body.</param>
    /// <param name="mass">Relative weight of the shape within its compound.</param>
    public static void GetBoxColliderSettings(SColliders.BoxCollider? collider,
        out Vector3 size,
        out Vector3 positionLocal,
        out Quaternion rotationLocal,
        out float mass)
    {
        size = collider?.Size ?? Vector3.One;
        positionLocal = collider?.PositionLocal ?? Vector3.Zero;
        rotationLocal = collider?.RotationLocal ?? Quaternion.Identity;
        mass = collider?.Mass ?? 1f;
    }

    /// <summary>Reads all settings of a sphere collider, mirroring the SphereCollider node's inputs.</summary>
    /// <param name="collider">The sphere collider to read. Outputs the defaults while null.</param>
    /// <param name="radius">Radius of the sphere.</param>
    /// <param name="positionLocal">Position of the shape relative to the body origin.</param>
    /// <param name="mass">Relative weight of the shape within its compound.</param>
    public static void GetSphereColliderSettings(SColliders.SphereCollider? collider,
        out float radius,
        out Vector3 positionLocal,
        out float mass)
    {
        radius = collider?.Radius ?? 0.5f;
        positionLocal = collider?.PositionLocal ?? Vector3.Zero;
        mass = collider?.Mass ?? 1f;
    }

    /// <summary>Reads all settings of a capsule collider, mirroring the CapsuleCollider node's inputs.</summary>
    /// <param name="collider">The capsule collider to read. Outputs the defaults while null.</param>
    /// <param name="radius">Radius of the capsule.</param>
    /// <param name="length">Length between the two cap centers.</param>
    /// <param name="positionLocal">Position of the shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of the shape relative to the body.</param>
    /// <param name="mass">Relative weight of the shape within its compound.</param>
    public static void GetCapsuleColliderSettings(SColliders.CapsuleCollider? collider,
        out float radius,
        out float length,
        out Vector3 positionLocal,
        out Quaternion rotationLocal,
        out float mass)
    {
        radius = collider?.Radius ?? 0.5f;
        length = collider?.Length ?? 1f;
        positionLocal = collider?.PositionLocal ?? Vector3.Zero;
        rotationLocal = collider?.RotationLocal ?? Quaternion.Identity;
        mass = collider?.Mass ?? 1f;
    }

    /// <summary>Reads all settings of a cylinder collider, mirroring the CylinderCollider node's inputs.</summary>
    /// <param name="collider">The cylinder collider to read. Outputs the defaults while null.</param>
    /// <param name="radius">Radius of the cylinder.</param>
    /// <param name="length">Height of the cylinder.</param>
    /// <param name="positionLocal">Position of the shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of the shape relative to the body.</param>
    /// <param name="mass">Relative weight of the shape within its compound.</param>
    public static void GetCylinderColliderSettings(SColliders.CylinderCollider? collider,
        out float radius,
        out float length,
        out Vector3 positionLocal,
        out Quaternion rotationLocal,
        out float mass)
    {
        radius = collider?.Radius ?? 0.5f;
        length = collider?.Length ?? 1f;
        positionLocal = collider?.PositionLocal ?? Vector3.Zero;
        rotationLocal = collider?.RotationLocal ?? Quaternion.Identity;
        mass = collider?.Mass ?? 1f;
    }

    /// <summary>Reads all settings of a triangle collider, mirroring the TriangleCollider node's inputs.</summary>
    /// <param name="collider">The triangle collider to read. Outputs the defaults while null.</param>
    /// <param name="a">First vertex relative to the body origin.</param>
    /// <param name="b">Second vertex relative to the body origin.</param>
    /// <param name="c">Third vertex relative to the body origin.</param>
    /// <param name="positionLocal">Position of the shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of the shape relative to the body.</param>
    /// <param name="mass">Relative weight of the shape within its compound.</param>
    public static void GetTriangleColliderSettings(SColliders.TriangleCollider? collider,
        out Vector3 a,
        out Vector3 b,
        out Vector3 c,
        out Vector3 positionLocal,
        out Quaternion rotationLocal,
        out float mass)
    {
        a = collider?.A ?? Vector3.Zero;
        b = collider?.B ?? Vector3.Zero;
        c = collider?.C ?? Vector3.Zero;
        positionLocal = collider?.PositionLocal ?? Vector3.Zero;
        rotationLocal = collider?.RotationLocal ?? Quaternion.Identity;
        mass = collider?.Mass ?? 1f;
    }
}
