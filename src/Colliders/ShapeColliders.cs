using System.ComponentModel;
using Stride.Core.Mathematics;
using VL.Core.Import;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
using SColliders = global::Stride.BepuPhysics.Definitions.Colliders;

namespace VL.Stride.BepuPhysics.Colliders;

/// <summary>Box collision shape.</summary>
[ProcessNode(Name = "BoxCollider")]
public class BoxColliderNode
{
    private readonly SColliders.BoxCollider _collider = new();
    private PinValue<Vector3> _size;
    private ColliderPinSync _sync;

    /// <param name="size">Extents of the box in meters. Every dimension must be greater than zero, a zero size box has zero inertia and produces NaN poses.</param>
    /// <param name="positionLocal">Position of this shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of this shape relative to the body. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="mass">Relative weight of this shape; distributes the compound inertia and center of mass. Must be greater than zero.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the collider again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SColliders.BoxCollider Update(
        [DefaultValue("1.0, 1.0, 1.0")] Vector3 size,
        Vector3 positionLocal,
        Quaternion rotationLocal,
        [DefaultValue(1f)] float mass,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        if (_size.Changed(size) | reapplyInputs)
            _collider.Size = size;
        _sync.Sync(_collider, positionLocal, rotationLocal, mass, reapplyInputs);
        return _collider;
    }
}

/// <summary>Sphere collision shape.</summary>
[ProcessNode(Name = "SphereCollider")]
public class SphereColliderNode
{
    private readonly SColliders.SphereCollider _collider = new();
    private PinValue<float> _radius;
    private ColliderPinSync _sync;

    /// <param name="radius">Radius of the sphere or capsule shape. Must be greater than zero.</param>
    /// <param name="positionLocal">Position of this shape relative to the body origin.</param>
    /// <param name="mass">Relative weight of this shape; distributes the compound inertia and center of mass. Must be greater than zero.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the collider again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SColliders.SphereCollider Update(
        [DefaultValue(0.5f)] float radius,
        Vector3 positionLocal,
        [DefaultValue(1f)] float mass,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        if (_radius.Changed(radius) | reapplyInputs)
            _collider.Radius = radius;
        _sync.Sync(_collider, positionLocal, Quaternion.Identity, mass, reapplyInputs);
        return _collider;
    }
}

/// <summary>Capsule collision shape (aligned to local Y).</summary>
[ProcessNode(Name = "CapsuleCollider")]
public class CapsuleColliderNode
{
    private readonly SColliders.CapsuleCollider _collider = new();
    private PinValue<float> _radius;
    private PinValue<float> _length;
    private ColliderPinSync _sync;

    /// <param name="radius">Radius of the sphere or capsule shape. Must be greater than zero.</param>
    /// <param name="length">Length between the two cap centers (total length = length + 2 * radius).</param>
    /// <param name="positionLocal">Position of this shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of this shape relative to the body. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="mass">Relative weight of this shape; distributes the compound inertia and center of mass. Must be greater than zero.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the collider again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SColliders.CapsuleCollider Update(
        [DefaultValue(0.5f)] float radius,
        [DefaultValue(1f)] float length,
        Vector3 positionLocal,
        Quaternion rotationLocal,
        [DefaultValue(1f)] float mass,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        if (_radius.Changed(radius) | reapplyInputs)
            _collider.Radius = radius;
        if (_length.Changed(length) | reapplyInputs)
            _collider.Length = length;
        _sync.Sync(_collider, positionLocal, rotationLocal, mass, reapplyInputs);
        return _collider;
    }
}

/// <summary>Cylinder collision shape (aligned to local Y).</summary>
[ProcessNode(Name = "CylinderCollider")]
public class CylinderColliderNode
{
    private readonly SColliders.CylinderCollider _collider = new();
    private PinValue<float> _radius;
    private PinValue<float> _length;
    private ColliderPinSync _sync;

    /// <param name="radius">Radius of the sphere or capsule shape. Must be greater than zero.</param>
    /// <param name="length">Height of the cylinder.</param>
    /// <param name="positionLocal">Position of this shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of this shape relative to the body. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="mass">Relative weight of this shape; distributes the compound inertia and center of mass. Must be greater than zero.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the collider again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SColliders.CylinderCollider Update(
        [DefaultValue(0.5f)] float radius,
        [DefaultValue(1f)] float length,
        Vector3 positionLocal,
        Quaternion rotationLocal,
        [DefaultValue(1f)] float mass,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        if (_radius.Changed(radius) | reapplyInputs)
            _collider.Radius = radius;
        if (_length.Changed(length) | reapplyInputs)
            _collider.Length = length;
        _sync.Sync(_collider, positionLocal, rotationLocal, mass, reapplyInputs);
        return _collider;
    }
}

/// <summary>Single-triangle collision shape.</summary>
[ProcessNode(Name = "TriangleCollider")]
public class TriangleColliderNode
{
    private readonly SColliders.TriangleCollider _collider = new();
    private PinValue<Vector3> _a;
    private PinValue<Vector3> _b;
    private PinValue<Vector3> _c;
    private ColliderPinSync _sync;

    /// <param name="a">First vertex relative to the body origin.</param>
    /// <param name="b">Second vertex relative to the body origin.</param>
    /// <param name="c">Third vertex relative to the body origin.</param>
    /// <param name="positionLocal">Position of this shape relative to the body origin.</param>
    /// <param name="rotationLocal">Rotation of this shape relative to the body. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="mass">Relative weight of this shape; distributes the compound inertia and center of mass. Must be greater than zero.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the collider again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SColliders.TriangleCollider Update(
        Vector3 a,
        Vector3 b,
        Vector3 c,
        Vector3 positionLocal,
        Quaternion rotationLocal,
        [DefaultValue(1f)] float mass,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        if (_a.Changed(a) | reapplyInputs)
            _collider.A = a;
        if (_b.Changed(b) | reapplyInputs)
            _collider.B = b;
        if (_c.Changed(c) | reapplyInputs)
            _collider.C = c;
        _sync.Sync(_collider, positionLocal, rotationLocal, mass, reapplyInputs);
        return _collider;
    }
}

/// <summary>
/// Per-node pin change detection for the placement properties shared by all shape colliders.
/// Like PinValue, diffs against the last pin value so external writes are not reverted.
/// </summary>
internal struct ColliderPinSync
{
    private PinValue<Vector3> _positionLocal;
    private PinValue<Quaternion> _rotationLocal;
    private PinValue<float> _mass;

    public void Sync(SColliders.ColliderBase collider, Vector3 positionLocal, Quaternion rotationLocal, float mass, bool force = false)
    {
        // Non-short-circuit | so the shadow fields update even while force is true.
        if (_positionLocal.Changed(positionLocal) | force)
            collider.PositionLocal = positionLocal;
        if (_rotationLocal.Changed(rotationLocal) | force)
            collider.RotationLocal = rotationLocal;
        if (_mass.Changed(mass) | force)
            collider.Mass = mass;
    }
}
