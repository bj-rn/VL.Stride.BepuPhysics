using VL.Core.Import;
using VL.Lib.Collections;
using SBepu = global::Stride.BepuPhysics;
using SColliders = global::Stride.BepuPhysics.Definitions.Colliders;
using SDefinitions = global::Stride.BepuPhysics.Definitions;
using SEngine = global::Stride.Engine;

namespace VL.Stride.BepuPhysics;

/// <summary>
/// Read and write access to collidables obtained from queries or contact events.
/// Use CastAs (BodyComponent) to narrow a CollidableComponent for the Body operations.
/// Mutating operations run while Apply is true — connect a Bang for one-shot application.
/// </summary>
public static class CollidableOperations
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
    public static void CollidableSettings(SBepu.CollidableComponent? collidable,
        out float springFrequency,
        out float springDampingRatio,
        out float frictionCoefficient,
        out float maximumRecoveryVelocity,
        out SBepu.CollisionLayer collisionLayer)
    {
        if (collidable is null)
        {
            springFrequency = 30f;
            springDampingRatio = 3f;
            frictionCoefficient = 1f;
            maximumRecoveryVelocity = 1000f;
            collisionLayer = SBepu.CollisionLayer.Layer0;
            return;
        }
        springFrequency = collidable.SpringFrequency;
        springDampingRatio = collidable.SpringDampingRatio;
        frictionCoefficient = collidable.FrictionCoefficient;
        maximumRecoveryVelocity = collidable.MaximumRecoveryVelocity;
        collisionLayer = collidable.CollisionLayer;
    }

    /// <summary>Reads the body specific settings. Use BodyState for pose and velocities.</summary>
    /// <param name="body">The body to read. Outputs the component defaults while null.</param>
    /// <param name="kinematic">Whether the body is kinematic (unaffected by forces and collisions).</param>
    /// <param name="sleepThreshold">Velocity below which the body becomes a sleep candidate; -1 disables sleeping.</param>
    /// <param name="gravity">Whether gravity affects this body. Only evaluated when UsePerBodyAttributes is enabled on the simulation.</param>
    /// <param name="interpolation">How the rendered motion is smoothed between fixed physics steps.</param>
    /// <param name="continuousDetection">Continuous collision detection mode of the body.</param>
    public static void BodySettings(SBepu.BodyComponent? body,
        out bool kinematic,
        out float sleepThreshold,
        out bool gravity,
        out SDefinitions.InterpolationMode interpolation,
        out ContinuousDetectionKind continuousDetection)
    {
        if (body is null)
        {
            kinematic = false;
            sleepThreshold = 0.01f;
            gravity = true;
            interpolation = SDefinitions.InterpolationMode.Interpolated;
            continuousDetection = ContinuousDetectionKind.Discrete;
            return;
        }
        kinematic = body.Kinematic;
        sleepThreshold = body.SleepThreshold;
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
    /// <param name="apply">Writes all settings each frame while true. Connect a Bang for a one-shot write.</param>
    [return: Pin(Name = "Output")]
    public static SBepu.CollidableComponent? SetCollidableSettings(SBepu.CollidableComponent? collidable,
        float springFrequency = 30f,
        float springDampingRatio = 3f,
        float frictionCoefficient = 1f,
        float maximumRecoveryVelocity = 1000f,
        SBepu.CollisionLayer? collisionLayer = null,
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
    /// <param name="gravity">Whether gravity affects this body. Only evaluated when UsePerBodyAttributes is enabled on the simulation.</param>
    /// <param name="interpolation">Smooths the rendered motion between fixed physics steps. Null = Interpolated.</param>
    /// <param name="continuousDetection">Continuous collision detection mode. Null = Discrete.</param>
    /// <param name="apply">Writes all settings each frame while true. Connect a Bang for a one-shot write.</param>
    [return: Pin(Name = "Output")]
    public static SBepu.BodyComponent? SetBodySettings(SBepu.BodyComponent? body,
        bool kinematic = false,
        float sleepThreshold = 0.01f,
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
[ProcessNode(Name = "GetColliders")]
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
