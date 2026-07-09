using VL.Lib.Collections;
using SColliders = global::Stride.BepuPhysics.Definitions.Colliders;

namespace VL.Stride.BepuPhysics.Internal;

/// <summary>
/// Shared collider-pin logic for Body/Static nodes: a pin group of shapes is wrapped in a
/// node-owned CompoundCollider; an explicit ICollider (MeshCollider, EmptyCollider) overrides it.
/// </summary>
internal sealed class ColliderInput
{
    private readonly SColliders.CompoundCollider _compound = new();
    private object? _lastShapes = new(); // sentinel so the first Update always syncs

    public SColliders.ICollider Resolve(Spread<SColliders.ColliderBase?>? shapes, SColliders.ICollider? colliderOverride)
    {
        if (colliderOverride is not null)
            return colliderOverride;

        // Spread is immutable — a content change always means a new instance.
        if (!ReferenceEquals(_lastShapes, shapes))
        {
            _lastShapes = shapes;
            // Each list mutation triggers the component's TryUpdateFeatures; changes are rare, so this is fine.
            _compound.Colliders.Clear();
            if (shapes is not null)
            {
                foreach (var shape in shapes)
                {
                    if (shape is not null)
                        _compound.Colliders.Add(shape);
                }
            }
        }
        // An empty compound simply doesn't attach — the legal "no shape yet" state.
        return _compound;
    }
}
