using VL.Core.Import;
using SColliders = global::Stride.BepuPhysics.Definitions.Colliders;
using SDefinitions = global::Stride.BepuPhysics.Definitions;
using SModel = global::Stride.Rendering.Model;

namespace VL.Stride.BepuPhysics.Colliders;

/// <summary>
/// Collision shape from a Stride Model's mesh data. Works with runtime/procedural models.
/// Connect to the Body/Static node's ColliderOverride pin (a mesh cannot be part of a compound).
/// Outputs null while no Model is connected.
/// </summary>
[ProcessNode(Name = "MeshCollider")]
public class MeshColliderNode
{
    private readonly SColliders.MeshCollider _collider = new() { Model = null! };

    /// <param name="model">Stride Model whose mesh data is used for collision. Works with runtime and procedural models.</param>
    /// <param name="closed">Whether the mesh is treated as a closed volume (enables correct inertia).</param>
    /// <param name="mass">Relative weight of this shape; distributes the compound inertia and center of mass. Must be greater than zero.</param>
    [return: Pin(Name = "Output")]
    public SColliders.ICollider? Update(
        SModel? model = null,
        bool closed = true,
        float mass = 1f)
    {
        if (model is null)
            return null;

        if (!ReferenceEquals(_collider.Model, model))
            _collider.Model = model;
        if (_collider.Closed != closed)
            _collider.Closed = closed;
        if (_collider.Mass != mass)
            _collider.Mass = mass;
        return _collider;
    }
}

/// <summary>
/// Convex hull collision shape from pre-decomposed hull data.
/// Outputs null while no hull data is connected. (Hull baking is not provided by this package.)
/// </summary>
[ProcessNode(Name = "ConvexHullCollider")]
public class ConvexHullColliderNode
{
    private readonly SColliders.ConvexHullCollider _collider = new() { Hull = null! };

    /// <param name="hull">Pre-decomposed hull data (DecomposedHulls asset). Outputs null while unconnected.</param>
    /// <param name="mass">Relative weight of this shape; distributes the compound inertia and center of mass. Must be greater than zero.</param>
    [return: Pin(Name = "Output")]
    public SColliders.ConvexHullCollider? Update(
        SDefinitions.DecomposedHulls? hull = null,
        float mass = 1f)
    {
        if (hull is null)
            return null;

        if (!ReferenceEquals(_collider.Hull, hull))
            _collider.Hull = hull;
        if (_collider.Mass != mass)
            _collider.Mass = mass;
        return _collider;
    }
}

/// <summary>
/// Explicit no-collision shape: the body participates in the simulation without colliding.
/// Connect to the Body node's ColliderOverride pin.
/// </summary>
[ProcessNode(Name = "EmptyCollider")]
public class EmptyColliderNode
{
    private readonly SColliders.EmptyCollider _collider = new();

    [return: Pin(Name = "Output")]
    public SColliders.ICollider Update() => _collider;
}
