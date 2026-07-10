using System.ComponentModel;
using Stride.Core.Mathematics;
using VL.Core.Import;
using SColliders = global::Stride.BepuPhysics.Definitions.Colliders;

namespace VL.Stride.BepuPhysics.Colliders;

/// <summary>Box collision shape.</summary>
[ProcessNode(Name = "BoxCollider")]
public class BoxColliderNode
{
    private readonly SColliders.BoxCollider _collider = new();

    /// <param name="size">Extents of the box in meters. Every dimension must be greater than zero, a zero size box has zero inertia and produces NaN poses.</param>
    /// <param name="positionLocal">Position of this shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of this shape relative to the body. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="mass">Relative weight of this shape; distributes the compound inertia and center of mass. Must be greater than zero.</param>
    [return: Pin(Name = "Output")]
    public SColliders.BoxCollider Update(
        [DefaultValue("1.0, 1.0, 1.0")] Vector3 size,
        Vector3 positionLocal,
        Quaternion rotationLocal,
        [DefaultValue(1f)] float mass)
    {
        if (_collider.Size != size)
            _collider.Size = size;
        ColliderCommon.Sync(_collider, positionLocal, rotationLocal, mass);
        return _collider;
    }
}

/// <summary>Sphere collision shape.</summary>
[ProcessNode(Name = "SphereCollider")]
public class SphereColliderNode
{
    private readonly SColliders.SphereCollider _collider = new();

    /// <param name="radius">Radius of the sphere or capsule shape. Must be greater than zero.</param>
    /// <param name="positionLocal">Position of this shape relative to the body origin.</param>
    /// <param name="mass">Relative weight of this shape; distributes the compound inertia and center of mass. Must be greater than zero.</param>
    [return: Pin(Name = "Output")]
    public SColliders.SphereCollider Update(
        [DefaultValue(0.5f)] float radius,
        Vector3 positionLocal,
        [DefaultValue(1f)] float mass)
    {
        if (_collider.Radius != radius)
            _collider.Radius = radius;
        ColliderCommon.Sync(_collider, positionLocal, Quaternion.Identity, mass);
        return _collider;
    }
}

/// <summary>Capsule collision shape (aligned to local Y).</summary>
[ProcessNode(Name = "CapsuleCollider")]
public class CapsuleColliderNode
{
    private readonly SColliders.CapsuleCollider _collider = new();

    /// <param name="radius">Radius of the sphere or capsule shape. Must be greater than zero.</param>
    /// <param name="length">Length between the two cap centers (total length = length + 2 * radius).</param>
    /// <param name="positionLocal">Position of this shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of this shape relative to the body. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="mass">Relative weight of this shape; distributes the compound inertia and center of mass. Must be greater than zero.</param>
    [return: Pin(Name = "Output")]
    public SColliders.CapsuleCollider Update(
        [DefaultValue(0.5f)] float radius,
        [DefaultValue(1f)] float length,
        Vector3 positionLocal,
        Quaternion rotationLocal,
        [DefaultValue(1f)] float mass)
    {
        if (_collider.Radius != radius)
            _collider.Radius = radius;
        if (_collider.Length != length)
            _collider.Length = length;
        ColliderCommon.Sync(_collider, positionLocal, rotationLocal, mass);
        return _collider;
    }
}

/// <summary>Cylinder collision shape (aligned to local Y).</summary>
[ProcessNode(Name = "CylinderCollider")]
public class CylinderColliderNode
{
    private readonly SColliders.CylinderCollider _collider = new();

    /// <param name="radius">Radius of the sphere or capsule shape. Must be greater than zero.</param>
    /// <param name="length">Height of the cylinder.</param>
    /// <param name="positionLocal">Position of this shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of this shape relative to the body. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="mass">Relative weight of this shape; distributes the compound inertia and center of mass. Must be greater than zero.</param>
    [return: Pin(Name = "Output")]
    public SColliders.CylinderCollider Update(
        [DefaultValue(0.5f)] float radius,
        [DefaultValue(1f)] float length,
        Vector3 positionLocal,
        Quaternion rotationLocal,
        [DefaultValue(1f)] float mass)
    {
        if (_collider.Radius != radius)
            _collider.Radius = radius;
        if (_collider.Length != length)
            _collider.Length = length;
        ColliderCommon.Sync(_collider, positionLocal, rotationLocal, mass);
        return _collider;
    }
}

/// <summary>Single-triangle collision shape.</summary>
[ProcessNode(Name = "TriangleCollider")]
public class TriangleColliderNode
{
    private readonly SColliders.TriangleCollider _collider = new();

    /// <param name="a">First vertex relative to the body origin.</param>
    /// <param name="b">Second vertex relative to the body origin.</param>
    /// <param name="c">Third vertex relative to the body origin.</param>
    /// <param name="positionLocal">Position of this shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of this shape relative to the body. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="mass">Relative weight of this shape; distributes the compound inertia and center of mass. Must be greater than zero.</param>
    [return: Pin(Name = "Output")]
    public SColliders.TriangleCollider Update(
        Vector3 a,
        Vector3 b,
        Vector3 c,
        Vector3 positionLocal,
        Quaternion rotationLocal,
        [DefaultValue(1f)] float mass)
    {
        if (_collider.A != a)
            _collider.A = a;
        if (_collider.B != b)
            _collider.B = b;
        if (_collider.C != c)
            _collider.C = c;
        ColliderCommon.Sync(_collider, positionLocal, rotationLocal, mass);
        return _collider;
    }
}

internal static class ColliderCommon
{
    public static void Sync(SColliders.ColliderBase collider, Vector3 positionLocal, Quaternion rotationLocal, float mass)
    {
        if (collider.PositionLocal != positionLocal)
            collider.PositionLocal = positionLocal;
        if (collider.RotationLocal != rotationLocal)
            collider.RotationLocal = rotationLocal;
        if (collider.Mass != mass)
            collider.Mass = mass;
    }
}
