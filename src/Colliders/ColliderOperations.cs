using System.ComponentModel;
using Stride.Core.Mathematics;
using VL.Core.Import;
using SColliders = global::Stride.BepuPhysics.Definitions.Colliders;

namespace VL.Stride.BepuPhysics.Colliders;

/// <summary>
/// Read and write access to collider shapes, for example from a GetColliders node.
/// Use CastAs (BoxCollider) to narrow a ColliderBase for the shape specific operations.
/// Mutating operations run while Apply is true — connect a Bang for one-shot application.
/// </summary>
public static class ColliderOperations
{
    /// <summary>Identifies which shape a collider is, for routing before CastAs.</summary>
    /// <param name="collider">The collider shape to identify.</param>
    /// <param name="kind">Which shape the collider is. None while nothing is connected.</param>
    public static void ColliderInfo(SColliders.ColliderBase? collider,
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
    public static void ColliderSettings(SColliders.ColliderBase? collider,
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
    public static void BoxColliderSettings(SColliders.BoxCollider? collider,
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
    public static void SphereColliderSettings(SColliders.SphereCollider? collider,
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
    public static void CapsuleColliderSettings(SColliders.CapsuleCollider? collider,
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
    public static void CylinderColliderSettings(SColliders.CylinderCollider? collider,
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
    public static void TriangleColliderSettings(SColliders.TriangleCollider? collider,
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

    /// <summary>
    /// Writes the placement properties shared by all collider shapes while Apply is true.
    /// A written property is overwritten again once the owning collider node's pin value changes.
    /// </summary>
    /// <param name="collider">The collider shape to write to.</param>
    /// <param name="positionLocal">Position of the shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of the shape relative to the body. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="mass">Relative weight of the shape within its compound. Must be greater than zero.</param>
    /// <param name="apply">Writes all settings each frame while true. Connect a Bang for a one-shot write.</param>
    [return: Pin(Name = "Output")]
    public static SColliders.ColliderBase? SetColliderSettings(SColliders.ColliderBase? collider,
        Vector3 positionLocal,
        Quaternion rotationLocal,
        [DefaultValue(1f)] float mass,
        bool apply = false)
    {
        if (apply && collider is not null)
        {
            collider.PositionLocal = positionLocal;
            collider.RotationLocal = rotationLocal;
            collider.Mass = mass;
        }
        return collider;
    }

    /// <summary>
    /// Writes all settings of a box collider while Apply is true, mirroring the BoxCollider node's inputs.
    /// A written property is overwritten again once the owning collider node's pin value changes.
    /// </summary>
    /// <param name="collider">The box collider to write to. Use CastAs (BoxCollider) to narrow a ColliderBase.</param>
    /// <param name="size">Extents of the box in meters. Every dimension must be greater than zero, a zero size box has zero inertia and produces NaN poses.</param>
    /// <param name="positionLocal">Position of the shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of the shape relative to the body. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="mass">Relative weight of the shape within its compound. Must be greater than zero.</param>
    /// <param name="apply">Writes all settings each frame while true. Connect a Bang for a one-shot write.</param>
    [return: Pin(Name = "Output")]
    public static SColliders.BoxCollider? SetBoxColliderSettings(SColliders.BoxCollider? collider,
        [DefaultValue("1.0, 1.0, 1.0")] Vector3 size,
        Vector3 positionLocal,
        Quaternion rotationLocal,
        [DefaultValue(1f)] float mass,
        bool apply = false)
    {
        if (apply && collider is not null)
        {
            collider.Size = size;
            collider.PositionLocal = positionLocal;
            collider.RotationLocal = rotationLocal;
            collider.Mass = mass;
        }
        return collider;
    }

    /// <summary>
    /// Writes all settings of a sphere collider while Apply is true, mirroring the SphereCollider node's inputs.
    /// A written property is overwritten again once the owning collider node's pin value changes.
    /// </summary>
    /// <param name="collider">The sphere collider to write to. Use CastAs (SphereCollider) to narrow a ColliderBase.</param>
    /// <param name="radius">Radius of the sphere. Must be greater than zero.</param>
    /// <param name="positionLocal">Position of the shape relative to the body origin.</param>
    /// <param name="mass">Relative weight of the shape within its compound. Must be greater than zero.</param>
    /// <param name="apply">Writes all settings each frame while true. Connect a Bang for a one-shot write.</param>
    [return: Pin(Name = "Output")]
    public static SColliders.SphereCollider? SetSphereColliderSettings(SColliders.SphereCollider? collider,
        [DefaultValue(0.5f)] float radius,
        Vector3 positionLocal,
        [DefaultValue(1f)] float mass,
        bool apply = false)
    {
        if (apply && collider is not null)
        {
            collider.Radius = radius;
            collider.PositionLocal = positionLocal;
            collider.Mass = mass;
        }
        return collider;
    }

    /// <summary>
    /// Writes all settings of a capsule collider while Apply is true, mirroring the CapsuleCollider node's inputs.
    /// A written property is overwritten again once the owning collider node's pin value changes.
    /// </summary>
    /// <param name="collider">The capsule collider to write to. Use CastAs (CapsuleCollider) to narrow a ColliderBase.</param>
    /// <param name="radius">Radius of the capsule. Must be greater than zero.</param>
    /// <param name="length">Length between the two cap centers.</param>
    /// <param name="positionLocal">Position of the shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of the shape relative to the body. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="mass">Relative weight of the shape within its compound. Must be greater than zero.</param>
    /// <param name="apply">Writes all settings each frame while true. Connect a Bang for a one-shot write.</param>
    [return: Pin(Name = "Output")]
    public static SColliders.CapsuleCollider? SetCapsuleColliderSettings(SColliders.CapsuleCollider? collider,
        [DefaultValue(0.5f)] float radius,
        [DefaultValue(1f)] float length,
        Vector3 positionLocal,
        Quaternion rotationLocal,
        [DefaultValue(1f)] float mass,
        bool apply = false)
    {
        if (apply && collider is not null)
        {
            collider.Radius = radius;
            collider.Length = length;
            collider.PositionLocal = positionLocal;
            collider.RotationLocal = rotationLocal;
            collider.Mass = mass;
        }
        return collider;
    }

    /// <summary>
    /// Writes all settings of a cylinder collider while Apply is true, mirroring the CylinderCollider node's inputs.
    /// A written property is overwritten again once the owning collider node's pin value changes.
    /// </summary>
    /// <param name="collider">The cylinder collider to write to. Use CastAs (CylinderCollider) to narrow a ColliderBase.</param>
    /// <param name="radius">Radius of the cylinder. Must be greater than zero.</param>
    /// <param name="length">Height of the cylinder.</param>
    /// <param name="positionLocal">Position of the shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of the shape relative to the body. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="mass">Relative weight of the shape within its compound. Must be greater than zero.</param>
    /// <param name="apply">Writes all settings each frame while true. Connect a Bang for a one-shot write.</param>
    [return: Pin(Name = "Output")]
    public static SColliders.CylinderCollider? SetCylinderColliderSettings(SColliders.CylinderCollider? collider,
        [DefaultValue(0.5f)] float radius,
        [DefaultValue(1f)] float length,
        Vector3 positionLocal,
        Quaternion rotationLocal,
        [DefaultValue(1f)] float mass,
        bool apply = false)
    {
        if (apply && collider is not null)
        {
            collider.Radius = radius;
            collider.Length = length;
            collider.PositionLocal = positionLocal;
            collider.RotationLocal = rotationLocal;
            collider.Mass = mass;
        }
        return collider;
    }

    /// <summary>
    /// Writes all settings of a triangle collider while Apply is true, mirroring the TriangleCollider node's inputs.
    /// A written property is overwritten again once the owning collider node's pin value changes.
    /// </summary>
    /// <param name="collider">The triangle collider to write to. Use CastAs (TriangleCollider) to narrow a ColliderBase.</param>
    /// <param name="a">First vertex relative to the body origin.</param>
    /// <param name="b">Second vertex relative to the body origin.</param>
    /// <param name="c">Third vertex relative to the body origin.</param>
    /// <param name="positionLocal">Position of the shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of the shape relative to the body. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="mass">Relative weight of the shape within its compound. Must be greater than zero.</param>
    /// <param name="apply">Writes all settings each frame while true. Connect a Bang for a one-shot write.</param>
    [return: Pin(Name = "Output")]
    public static SColliders.TriangleCollider? SetTriangleColliderSettings(SColliders.TriangleCollider? collider,
        Vector3 a,
        Vector3 b,
        Vector3 c,
        Vector3 positionLocal,
        Quaternion rotationLocal,
        [DefaultValue(1f)] float mass,
        bool apply = false)
    {
        if (apply && collider is not null)
        {
            collider.A = a;
            collider.B = b;
            collider.C = c;
            collider.PositionLocal = positionLocal;
            collider.RotationLocal = rotationLocal;
            collider.Mass = mass;
        }
        return collider;
    }
}
