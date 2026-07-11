using VL.Core.Import;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
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
    private PinValue<SModel?> _model;
    private PinValue<bool> _closed;
    private PinValue<float> _mass;

    /// <param name="model">Stride Model whose mesh data is used for collision. Works with runtime and procedural models.</param>
    /// <param name="closed">Whether the mesh is treated as a closed volume (enables correct inertia).</param>
    /// <param name="mass">Relative weight of this shape; distributes the compound inertia and center of mass. Must be greater than zero.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the collider again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SColliders.ICollider? Update(
        SModel? model = null,
        bool closed = true,
        float mass = 1f,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        if (model is null)
            return null;

        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_model.Changed(model) | reapplyInputs)
            _collider.Model = model;
        if (_closed.Changed(closed) | reapplyInputs)
            _collider.Closed = closed;
        if (_mass.Changed(mass) | reapplyInputs)
            _collider.Mass = mass;
        return _collider;
    }
}

/// <summary>
/// Convex hull collision shape. Bake the hull data at runtime with HullFromModel or
/// HullFromPoints. Outputs null while no hull data is connected.
/// </summary>
[ProcessNode(Name = "ConvexHullCollider")]
public class ConvexHullColliderNode
{
    private readonly SColliders.ConvexHullCollider _collider = new() { Hull = null! };
    private PinValue<SDefinitions.DecomposedHulls?> _hull;
    private PinValue<float> _mass;

    /// <param name="hull">Hull data, from a HullFromModel or HullFromPoints node. Outputs null while unconnected.</param>
    /// <param name="mass">Relative weight of this shape; distributes the compound inertia and center of mass. Must be greater than zero.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the collider again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SColliders.ConvexHullCollider? Update(
        SDefinitions.DecomposedHulls? hull = null,
        float mass = 1f,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        if (hull is null)
            return null;

        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_hull.Changed(hull) | reapplyInputs)
            _collider.Hull = hull;
        if (_mass.Changed(mass) | reapplyInputs)
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
