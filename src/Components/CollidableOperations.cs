using Stride.Core.Mathematics;
using VL.Core.Import;
using VL.Lib.Collections;
using SBepu = global::Stride.BepuPhysics;
using SColliders = global::Stride.BepuPhysics.Definitions.Colliders;
using SDefinitions = global::Stride.BepuPhysics.Definitions;
using SEngine = global::Stride.Engine;

namespace VL.Stride.BepuPhysics;

// Collidable level operations of the Bodies category (see BodyOperations.cs for the summary).
// Use CastAs (BodyComponent) to narrow a CollidableComponent for the Body operations.
public static partial class Bodies
{
    /// <summary>Identifies a collidable: its entity and whether it is a body or a static.</summary>
    /// <param name="collidable">The collidable to identify, for example from a RayCast hit or a ContactEvents contact.</param>
    /// <param name="entity">The entity the collidable is attached to. Null while not attached or no input.</param>
    /// <param name="entityName">Name of that entity. Empty while not attached or no input.</param>
    /// <param name="kind">Whether the collidable is a Body, a Static or nothing is connected.</param>
    /// <param name="attached">True while the collidable is part of a simulation.</param>
    public static void CollidableInfo(SBepu.CollidableComponent? collidable,
        out SEngine.Entity? entity,
        out string entityName,
        out CollidableKind kind,
        out bool attached)
    {
        entity = collidable?.Entity;
        entityName = entity?.Name ?? string.Empty;
        kind = collidable switch
        {
            SBepu.BodyComponent => CollidableKind.Body,
            SBepu.StaticComponent => CollidableKind.Static,
            _ => CollidableKind.None,
        };
        attached = collidable?.Simulation is not null;
    }

    /// <summary>Reads the contact material settings shared by bodies and statics.</summary>
    /// <param name="collidable">The collidable to read. Outputs the component defaults while null.</param>
    /// <param name="springFrequency">Contact spring stiffness in Hz.</param>
    /// <param name="springDampingRatio">Contact spring damping; 1 = critical damping.</param>
    /// <param name="frictionCoefficient">Surface friction; 0 = frictionless, 1 = rough.</param>
    /// <param name="maximumRecoveryVelocity">Upper limit for the velocity used to push overlapping bodies apart.</param>
    /// <param name="collisionLayer">The collision layer of this collidable (0..31).</param>
    /// <param name="collisionGroup">The collision group of this collidable. Use Split (CollisionGroup) to access its parts.</param>
    public static void CollidableSettings(SBepu.CollidableComponent? collidable,
        out float springFrequency,
        out float springDampingRatio,
        out float frictionCoefficient,
        out float maximumRecoveryVelocity,
        out SBepu.CollisionLayer collisionLayer,
        out SDefinitions.CollisionGroup collisionGroup)
    {
        if (collidable is null)
        {
            springFrequency = 30f;
            springDampingRatio = 3f;
            frictionCoefficient = 1f;
            maximumRecoveryVelocity = 1000f;
            collisionLayer = SBepu.CollisionLayer.Layer0;
            collisionGroup = default;
            return;
        }
        springFrequency = collidable.SpringFrequency;
        springDampingRatio = collidable.SpringDampingRatio;
        frictionCoefficient = collidable.FrictionCoefficient;
        maximumRecoveryVelocity = collidable.MaximumRecoveryVelocity;
        collisionLayer = collidable.CollisionLayer;
        collisionGroup = collidable.CollisionGroup;
    }

    /// <summary>
    /// Creates a collision group, a fine grained filter on top of the collision layer:
    /// collidables sharing the same non zero Id do not collide while the absolute difference
    /// of their IndexA, IndexB and IndexC is less than two.
    /// Example teams: characters A, B, C, D on one layer, split into teams {A, B} and {C, D};
    /// to stop team members from colliding with each other, give A and B the Id 1 and give
    /// C and D the Id 2 (all indices 0).
    /// Example chain: colliders A, B, C attached in a row; B should not collide with its
    /// neighbours A and C, but A and C should collide with each other. Give all three the
    /// same Id and set Index A to 0, 1 and 2: A and C collide since their difference is two,
    /// neither collides with B since both are only one away from B's index.
    /// </summary>
    /// <param name="id">The group identification number; 0 = no filtering beyond the collision layer (0..65535).</param>
    /// <param name="indexA">Index of this collidable within the group; same or adjacent values ignore each other, values two or more apart collide (0..65535).</param>
    /// <param name="indexB">Second index, same rule as Index A. All three index differences must be less than two to suppress a collision.</param>
    /// <param name="indexC">Third index, same rule as Index A.</param>
    public static SDefinitions.CollisionGroup CollisionGroup(int id = 0, int indexA = 0, int indexB = 0, int indexC = 0)
    {
        return new SDefinitions.CollisionGroup
        {
            Id = (ushort)Math.Clamp(id, ushort.MinValue, ushort.MaxValue),
            IndexA = (ushort)Math.Clamp(indexA, ushort.MinValue, ushort.MaxValue),
            IndexB = (ushort)Math.Clamp(indexB, ushort.MinValue, ushort.MaxValue),
            IndexC = (ushort)Math.Clamp(indexC, ushort.MinValue, ushort.MaxValue),
        };
    }

    /// <summary>Splits a collision group into its parts.</summary>
    /// <param name="input">The collision group to split.</param>
    /// <param name="id">The group identification number; 0 = no filtering beyond the collision layer.</param>
    /// <param name="indexA">Index of the collidable within the group.</param>
    /// <param name="indexB">Second index.</param>
    /// <param name="indexC">Third index.</param>
    [Name("Split (CollisionGroup)")]
    public static void Split(SDefinitions.CollisionGroup? input,
        out int id, out int indexA, out int indexB, out int indexC)
    {
        var group = input ?? default;
        id = group.Id;
        indexA = group.IndexA;
        indexB = group.IndexB;
        indexC = group.IndexC;
    }

    /// <summary>
    /// Casts a ray against a single collidable, ignoring everything else in the simulation.
    /// Cheaper and more precise than a scene wide query when only one object matters,
    /// for example picking against a specific body.
    /// </summary>
    /// <param name="collidable">The collidable to test. No hit while null or not attached to a simulation.</param>
    /// <param name="hit">The closest hit: point, normal, distance and child index.</param>
    /// <param name="didHit">True when the ray hit the collidable within Max Distance.</param>
    /// <param name="origin">Ray start position in world space.</param>
    /// <param name="direction">Ray direction in world space; its length does not matter (normalized internally, distances are world units).</param>
    /// <param name="maxDistance">Maximum travel distance of the ray.</param>
    public static void RayCast(SBepu.CollidableComponent? collidable,
        out SBepu.HitInfo hit,
        out bool didHit,
        Vector3 origin,
        Vector3 direction,
        float maxDistance = 100f)
    {
        if (collidable is null || collidable.Simulation is null || direction == Vector3.Zero)
        {
            hit = default;
            didHit = false;
            return;
        }
        // Bepu measures maxDistance and the hit T in units of the direction's LENGTH —
        // normalize so both are plain world units no matter what is connected.
        direction.Normalize();
        didHit = collidable.RayCast(origin, direction, maxDistance, out hit);
    }

    /// <summary>Reads the body specific settings. Use BodyState for pose and velocities.</summary>
    /// <param name="body">The body to read. Outputs the component defaults while null.</param>
    /// <param name="kinematic">Whether the body is kinematic (unaffected by forces and collisions).</param>
    /// <param name="sleepThreshold">Velocity below which the body becomes a sleep candidate; -1 disables sleeping.</param>
    /// <param name="minimumTimestepCountUnderThreshold">Number of physics steps the body must stay under the sleep threshold before it becomes a sleeping candidate.</param>
    /// <param name="gravity">Whether gravity affects this body. Only evaluated when UsePerBodyAttributes is enabled on the simulation.</param>
    /// <param name="interpolation">How the rendered motion is smoothed between fixed physics steps.</param>
    /// <param name="continuousDetection">Continuous collision detection mode of the body.</param>
    public static void BodySettings(SBepu.BodyComponent? body,
        out bool kinematic,
        out float sleepThreshold,
        out int minimumTimestepCountUnderThreshold,
        out bool gravity,
        out SDefinitions.InterpolationMode interpolation,
        out ContinuousDetectionKind continuousDetection)
    {
        if (body is null)
        {
            kinematic = false;
            sleepThreshold = 0.01f;
            minimumTimestepCountUnderThreshold = 32;
            gravity = true;
            interpolation = SDefinitions.InterpolationMode.Interpolated;
            continuousDetection = ContinuousDetectionKind.Discrete;
            return;
        }
        kinematic = body.Kinematic;
        sleepThreshold = body.SleepThreshold;
        minimumTimestepCountUnderThreshold = body.MinimumTimestepCountUnderThreshold;
        gravity = body.Gravity;
        interpolation = body.InterpolationMode;
        continuousDetection = (ContinuousDetectionKind)body.ContinuousDetectionMode;
    }

    /// <summary>
    /// Writes the contact material settings shared by bodies and statics while Apply is true.
    /// A written property is overwritten again once the owning Body or Static node's pin value changes.
    /// </summary>
    /// <param name="collidable">The collidable to write to, for example from a RayCast hit or a ContactEvents contact.</param>
    /// <param name="springFrequency">Contact spring stiffness in Hz.</param>
    /// <param name="springDampingRatio">Contact spring damping; 1 = critical damping.</param>
    /// <param name="frictionCoefficient">Surface friction; 0 = frictionless, 1 = rough.</param>
    /// <param name="maximumRecoveryVelocity">Upper limit for the velocity used to push overlapping bodies apart.</param>
    /// <param name="collisionLayer">The collision layer of this collidable (0..31). Null = Layer0.</param>
    /// <param name="collisionGroup">Fine grained filter on top of the collision layer. Create with the CollisionGroup operation. Null = no group.</param>
    /// <param name="apply">Writes all settings each frame while true. Connect a Bang for a one-shot write.</param>
    [return: Pin(Name = "Output")]
    public static SBepu.CollidableComponent? SetCollidableSettings(SBepu.CollidableComponent? collidable,
        float springFrequency = 30f,
        float springDampingRatio = 3f,
        float frictionCoefficient = 1f,
        float maximumRecoveryVelocity = 1000f,
        SBepu.CollisionLayer? collisionLayer = null,
        SDefinitions.CollisionGroup? collisionGroup = null,
        bool apply = false)
    {
        if (apply && collidable is not null)
        {
            // Wake first — mutating a sleeping body writes into sleeping-set memory.
            if (collidable is SBepu.BodyComponent body)
                body.Awake = true;
            collidable.SpringFrequency = springFrequency;
            collidable.SpringDampingRatio = springDampingRatio;
            collidable.FrictionCoefficient = frictionCoefficient;
            collidable.MaximumRecoveryVelocity = maximumRecoveryVelocity;
            collidable.CollisionLayer = collisionLayer ?? SBepu.CollisionLayer.Layer0;
            collidable.CollisionGroup = collisionGroup ?? default;
        }
        return collidable;
    }

    /// <summary>
    /// Writes the body specific settings while Apply is true.
    /// A written property is overwritten again once the owning Body node's pin value changes.
    /// </summary>
    /// <param name="body">The body to write to. Use CastAs (BodyComponent) to narrow a collidable.</param>
    /// <param name="kinematic">When true the body is unaffected by forces and collisions but pushes dynamic bodies away.</param>
    /// <param name="sleepThreshold">Velocity below which the body becomes a sleep candidate; -1 disables sleeping.</param>
    /// <param name="minimumTimestepCountUnderThreshold">Number of physics steps the body must stay under the sleep threshold before it becomes a sleeping candidate (1..255).</param>
    /// <param name="gravity">Whether gravity affects this body. Only evaluated when UsePerBodyAttributes is enabled on the simulation.</param>
    /// <param name="interpolation">Smooths the rendered motion between fixed physics steps. Null = Interpolated.</param>
    /// <param name="continuousDetection">Continuous collision detection mode. Null = Discrete.</param>
    /// <param name="apply">Writes all settings each frame while true. Connect a Bang for a one-shot write.</param>
    [return: Pin(Name = "Output")]
    public static SBepu.BodyComponent? SetBodySettings(SBepu.BodyComponent? body,
        bool kinematic = false,
        float sleepThreshold = 0.01f,
        int minimumTimestepCountUnderThreshold = 32,
        bool gravity = true,
        SDefinitions.InterpolationMode? interpolation = null,
        ContinuousDetectionKind? continuousDetection = null,
        bool apply = false)
    {
        if (apply && body is not null)
        {
            // Wake first — mutating a sleeping body writes into sleeping-set memory.
            body.Awake = true;
            body.Kinematic = kinematic;
            body.SleepThreshold = sleepThreshold;
            body.MinimumTimestepCountUnderThreshold = (byte)Math.Clamp(minimumTimestepCountUnderThreshold, 1, byte.MaxValue);
            body.Gravity = gravity;
            body.InterpolationMode = interpolation ?? SDefinitions.InterpolationMode.Interpolated;
            body.ContinuousDetectionMode = (global::BepuPhysics.Collidables.ContinuousDetectionMode)(continuousDetection ?? ContinuousDetectionKind.Discrete);
        }
        return body;
    }
}

/// <summary>
/// Reads the collider shapes of a collidable's compound.
/// Connect a RayCast hit's ChildIndex to pick the exact shape that was hit.
/// </summary>
[ProcessNode(Name = "GetColliders", Category = "Stride.Physics.Bepu.Bodies")]
public class GetCollidersNode
{
    private readonly SpreadBuilder<SColliders.ColliderBase> _builder = new();
    private Spread<SColliders.ColliderBase> _colliders = Spread<SColliders.ColliderBase>.Empty;

    /// <param name="collidable">The collidable whose shapes are read, for example from a RayCast hit or a ContactEvents contact.</param>
    /// <param name="child">The shape at Child Index. Null while the index is out of range. Use CastAs (BoxCollider) to access shape properties.</param>
    /// <param name="childIndex">Index of the shape to output on the Child pin. A HitInfo's ChildIndex connects here.</param>
    /// <returns>All collider shapes of the collidable's compound. Empty for mesh, empty and unconnected collidables.</returns>
    [return: Pin(Name = "Output")]
    public Spread<SColliders.ColliderBase> Update(
        SBepu.CollidableComponent? collidable,
        out SColliders.ColliderBase? child,
        int childIndex = 0)
    {
        var compound = collidable?.Collider as SColliders.CompoundCollider;
        if (!Matches(_colliders, compound))
        {
            _builder.Clear();
            if (compound is not null)
            {
                var list = compound.Colliders;
                for (var i = 0; i < list.Count; i++)
                    _builder.Add(list[i]);
            }
            _colliders = _builder.ToSpread();
        }
        child = childIndex >= 0 && childIndex < _colliders.Count ? _colliders[childIndex] : null;
        return _colliders;
    }

    private static bool Matches(Spread<SColliders.ColliderBase> spread, SColliders.CompoundCollider? compound)
    {
        if (compound is null)
            return spread.Count == 0;
        var list = compound.Colliders;
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
/// Casts a ray against a single collidable and reports all hits along it, for example the
/// entry and exit points through its shapes. Everything else in the simulation is ignored.
/// </summary>
[ProcessNode(Name = "RayCastPenetrating", Category = "Stride.Physics.Bepu.Bodies")]
public class CollidableRayCastPenetratingNode
{
    // SpreadBuilder implements ICollection<T>, the engine appends hits directly into it.
    private readonly SpreadBuilder<SBepu.HitInfo> _builder = new();
    private Spread<SBepu.HitInfo> _result = Spread<SBepu.HitInfo>.Empty;

    /// <param name="collidable">The collidable to test. Empty while null or not attached to a simulation.</param>
    /// <param name="origin">Ray start position in world space.</param>
    /// <param name="direction">Ray direction in world space; its length does not matter (normalized internally, distances are world units).</param>
    /// <param name="maxDistance">Maximum travel distance of the ray.</param>
    /// <param name="enabled">Skips the query and outputs an empty spread when false.</param>
    [return: Pin(Name = "Output")]
    public Spread<SBepu.HitInfo> Update(
        SBepu.CollidableComponent? collidable,
        Vector3 origin,
        Vector3 direction,
        float maxDistance = 100f,
        bool enabled = true)
    {
        if (!enabled || collidable is null || collidable.Simulation is null || direction == Vector3.Zero)
            return _result = Spread<SBepu.HitInfo>.Empty;

        // Bepu measures maxDistance and the hit T in units of the direction's LENGTH —
        // normalize so both are plain world units no matter what is connected.
        direction.Normalize();
        _builder.Clear();
        collidable.RayCastPenetrating(origin, direction, maxDistance, _builder);
        return _result = _builder.Count == 0 ? Spread<SBepu.HitInfo>.Empty : _builder.ToSpread();
    }
}
