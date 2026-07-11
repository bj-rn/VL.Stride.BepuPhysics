using System.ComponentModel;
using BepuPhysics.Collidables;
using Stride.Core.Mathematics;
using VL.Core.Import;
using VL.Lib.Collections;
using SBepu = global::Stride.BepuPhysics;
using SDefinitions = global::Stride.BepuPhysics.Definitions;

namespace VL.Stride.BepuPhysics.Queries;

/// <summary>
/// Stateless physics queries.
/// </summary>
public static class QueryOperations
{
    /// <summary>Splits a query hit into its parts.</summary>
    /// <param name="input">The hit to split.</param>
    /// <param name="point">The position where the intersection occurred, in world space.</param>
    /// <param name="normal">The surface normal at the hit, in world space.</param>
    /// <param name="distance">The distance along the ray or sweep where the hit occurred.</param>
    /// <param name="collidable">The Body or Static component that was hit.</param>
    /// <param name="childIndex">Index of the child shape that was hit when the collidable uses a compound collider.</param>
    public static void Split(SBepu.HitInfo? input,
        out Vector3 point,
        out Vector3 normal,
        out float distance,
        out SBepu.CollidableComponent? collidable,
        out int childIndex)
    {
        point = input?.Point ?? default;
        normal = input?.Normal ?? default;
        distance = input?.Distance ?? default;
        collidable = input?.Collidable;
        childIndex = input?.ChildIndex ?? -1;
    }

    /// <summary>Splits an overlap query result into its parts.</summary>
    /// <param name="input">The overlap to split.</param>
    /// <param name="collidable">The Body or Static component the test shape overlaps with.</param>
    /// <param name="penetrationDirection">Direction the test shape has to move along to separate from this overlap.</param>
    /// <param name="penetrationLength">Distance the test shape has to move to separate from this overlap.</param>
    public static void Split(SBepu.OverlapInfo? input,
        out SBepu.CollidableComponent? collidable,
        out Vector3 penetrationDirection,
        out float penetrationLength)
    {
        collidable = input?.Collidable;
        penetrationDirection = input?.PenetrationDirection ?? default;
        penetrationLength = input?.PenetrationLength ?? default;
    }

    /// <summary>Casts a ray and reports the closest hit.</summary>
    /// <param name="simulation">The simulation to query - from a SimulationSettings or GetSimulation node.</param>
    /// <param name="hit">The closest hit: point, normal, distance and the collidable that was hit.</param>
    /// <param name="didHit">True when the ray hit anything within Max Distance.</param>
    /// <param name="origin">Ray start position in world space.</param>
    /// <param name="direction">Ray direction in world space; its length does not matter (normalized internally, distances are world units).</param>
    /// <param name="maxDistance">Maximum travel distance of the ray.</param>
    /// <param name="collisionMask">Which collision layers the ray tests against. Null = Everything.</param>
    public static void RayCast(
        SBepu.BepuSimulation? simulation,
        out SBepu.HitInfo hit,
        out bool didHit,
        Vector3 origin,
        Vector3 direction,
        float maxDistance = 100f,
        SBepu.CollisionMask? collisionMask = null)
    {
        if (simulation is null || direction == Vector3.Zero)
        {
            hit = default;
            didHit = false;
            return;
        }
        // Bepu measures maxDistance and the hit T in units of the direction's LENGTH —
        // normalize so both are plain world units no matter what is connected.
        direction.Normalize();
        didHit = simulation.RayCast(origin, direction, maxDistance, out hit, collisionMask ?? SBepu.CollisionMask.Everything);
    }

    /// <summary>Sweeps a shape along a direction and reports the closest hit.</summary>
    /// <param name="simulation">The simulation to query - from a SimulationSettings or GetSimulation node.</param>
    /// <param name="hit">The closest hit: point, normal, distance and the collidable that was hit.</param>
    /// <param name="didHit">True when the sweep hit anything within Max Distance.</param>
    /// <param name="origin">Sweep start position in world space.</param>
    /// <param name="direction">Sweep direction in world space; its length does not matter (normalized internally, distances are world units).</param>
    /// <param name="maxDistance">Maximum travel distance of the sweep.</param>
    /// <param name="shape">Shape used by the query. Null = Sphere.</param>
    /// <param name="radius">Radius of the sphere or capsule shape. Must be greater than zero.</param>
    /// <param name="boxSize">Extents of the box shape. Every dimension must be greater than zero.</param>
    /// <param name="capsuleLength">Length of the capsule shape between the cap centers.</param>
    /// <param name="orientation">Orientation of the swept shape. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="collisionMask">Which collision layers the query tests against. Null = Everything.</param>
    public static void SweepCast(
        SBepu.BepuSimulation? simulation,
        out SBepu.HitInfo hit,
        out bool didHit,
        Vector3 origin,
        Vector3 direction,
        [DefaultValue(100f)] float maxDistance,
        SweepShape? shape,
        [DefaultValue(0.5f)] float radius,
        [Pin(Name = "Box Size"), DefaultValue("1.0, 1.0, 1.0")] Vector3 boxSize,
        [DefaultValue(1f)] float capsuleLength,
        Quaternion orientation,
        SBepu.CollisionMask? collisionMask)
    {
        if (simulation is null || direction == Vector3.Zero)
        {
            hit = default;
            didHit = false;
            return;
        }
        var mask = collisionMask ?? SBepu.CollisionMask.Everything;
        // Bepu measures maxDistance and the hit T in units of the direction's LENGTH —
        // normalize so both are plain world units no matter what is connected.
        direction.Normalize();
        var pose = new SDefinitions.RigidPose(origin, orientation);
        var velocity = new SDefinitions.BodyVelocity(direction, Vector3.Zero);
        switch (shape ?? SweepShape.Sphere)
        {
            case SweepShape.Sphere:
                didHit = simulation.SweepCast(new Sphere(radius), pose, velocity, maxDistance, out hit, mask);
                break;
            case SweepShape.Box:
                didHit = simulation.SweepCast(new Box(boxSize.X, boxSize.Y, boxSize.Z), pose, velocity, maxDistance, out hit, mask);
                break;
            case SweepShape.Capsule:
                didHit = simulation.SweepCast(new Capsule(radius, capsuleLength), pose, velocity, maxDistance, out hit, mask);
                break;
            default:
                hit = default;
                didHit = false;
                break;
        }
    }
}

/// <summary>
/// Casts a ray and reports all hits along it, sorted by distance.
/// </summary>
[ProcessNode(Name = "RayCastPenetrating")]
public class RayCastPenetratingNode
{
    // SpreadBuilder implements ICollection<T>, the engine appends hits directly into it.
    private readonly SpreadBuilder<SBepu.HitInfo> _builder = new();
    private Spread<SBepu.HitInfo> _result = Spread<SBepu.HitInfo>.Empty;

    /// <param name="simulation">The simulation to query — from a SimulationSettings or GetSimulation node.</param>
    /// <param name="origin">Ray start position in world space.</param>
    /// <param name="direction">Ray direction in world space; its length does not matter (normalized internally, distances are world units).</param>
    /// <param name="maxDistance">Maximum travel distance of the query.</param>
    /// <param name="collisionMask">Which collision layers the query tests against. Null = Everything.</param>
    /// <param name="enabled">Skips the query and outputs an empty spread when false.</param>
    [return: Pin(Name = "Output")]
    public Spread<SBepu.HitInfo> Update(
        SBepu.BepuSimulation? simulation,
        Vector3 origin,
        Vector3 direction,
        float maxDistance = 100f,
        SBepu.CollisionMask? collisionMask = null,
        bool enabled = true)
    {
        if (!enabled || simulation is null || direction == Vector3.Zero)
            return _result = Spread<SBepu.HitInfo>.Empty;

        // Bepu measures maxDistance and the hit T in units of the direction's LENGTH —
        // normalize so both are plain world units no matter what is connected.
        direction.Normalize();
        _builder.Clear();
        simulation.RayCastPenetrating(origin, direction, maxDistance, _builder, collisionMask ?? SBepu.CollisionMask.Everything);
        return _result = _builder.Count == 0 ? Spread<SBepu.HitInfo>.Empty : _builder.ToSpread();
    }
}

/// <summary>
/// Sweeps a shape along a direction and reports all hits.
/// </summary>
[ProcessNode(Name = "SweepCastPenetrating")]
public class SweepCastPenetratingNode
{
    // SpreadBuilder implements ICollection<T>, the engine appends hits directly into it.
    private readonly SpreadBuilder<SBepu.HitInfo> _builder = new();
    private Spread<SBepu.HitInfo> _result = Spread<SBepu.HitInfo>.Empty;

    /// <param name="simulation">The simulation to query — from a SimulationSettings or GetSimulation node.</param>
    /// <param name="origin">Sweep start position in world space.</param>
    /// <param name="direction">Sweep direction in world space; its length does not matter (normalized internally, distances are world units).</param>
    /// <param name="maxDistance">Maximum travel distance of the query.</param>
    /// <param name="shape">Shape used by the query. Null = Sphere.</param>
    /// <param name="radius">Radius of the sphere or capsule shape. Must be greater than zero.</param>
    /// <param name="boxSize">Extents of the box shape. Every dimension must be greater than zero.</param>
    /// <param name="capsuleLength">Length of the capsule shape between the cap centers.</param>
    /// <param name="orientation">Orientation of the swept shape. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="collisionMask">Which collision layers the query tests against. Null = Everything.</param>
    /// <param name="enabled">Skips the query and outputs an empty spread when false.</param>
    [return: Pin(Name = "Output")]
    public Spread<SBepu.HitInfo> Update(
        SBepu.BepuSimulation? simulation,
        Vector3 origin,
        Vector3 direction,
        [DefaultValue(100f)] float maxDistance,
        SweepShape? shape,
        [DefaultValue(0.5f)] float radius,
        [Pin(Name = "Box Size"), DefaultValue("1.0, 1.0, 1.0")] Vector3 boxSize,
        [DefaultValue(1f)] float capsuleLength,
        Quaternion orientation,
        SBepu.CollisionMask? collisionMask,
        bool enabled = true)
    {
        if (!enabled || simulation is null || direction == Vector3.Zero)
            return _result = Spread<SBepu.HitInfo>.Empty;


        var mask = collisionMask ?? SBepu.CollisionMask.Everything;
        // Bepu measures maxDistance and the hit T in units of the direction's LENGTH —
        // normalize so both are plain world units no matter what is connected.
        direction.Normalize();
        _builder.Clear();
        var pose = new SDefinitions.RigidPose(origin, orientation);
        var velocity = new SDefinitions.BodyVelocity(direction, Vector3.Zero);
        switch (shape ?? SweepShape.Sphere)
        {
            case SweepShape.Sphere:
                simulation.SweepCastPenetrating(new Sphere(radius), pose, velocity, maxDistance, _builder, mask);
                break;
            case SweepShape.Box:
                simulation.SweepCastPenetrating(new Box(boxSize.X, boxSize.Y, boxSize.Z), pose, velocity, maxDistance, _builder, mask);
                break;
            case SweepShape.Capsule:
                simulation.SweepCastPenetrating(new Capsule(radius, capsuleLength), pose, velocity, maxDistance, _builder, mask);
                break;
        }
        return _result = _builder.Count == 0 ? Spread<SBepu.HitInfo>.Empty : _builder.ToSpread();
    }
}

/// <summary>
/// Reports all overlaps of a shape placed at a position: the overlapping collidables plus
/// the direction and distance needed to separate. A compound collidable can produce several
/// overlaps, one per overlapping child shape. Use Split to access an overlap's parts.
/// </summary>
[ProcessNode(Name = "Overlap")]
public class OverlapNode
{
    // SpreadBuilder implements ICollection<T>, the engine appends overlaps directly into it.
    private readonly SpreadBuilder<SBepu.OverlapInfo> _infoBuilder = new();
    private readonly SpreadBuilder<SBepu.CollidableComponent> _collidableBuilder = new();
    private Spread<SBepu.OverlapInfo> _result = Spread<SBepu.OverlapInfo>.Empty;
    private Spread<SBepu.CollidableComponent> _collidables = Spread<SBepu.CollidableComponent>.Empty;

    /// <param name="simulation">The simulation to query — from a SimulationSettings or GetSimulation node.</param>
    /// <param name="collidables">The collidable of each overlap, in the same order as the Output. May contain a collidable several times when it uses a compound collider.</param>
    /// <param name="position">Center of the test shape in world space.</param>
    /// <param name="shape">Shape used by the query. Null = Sphere.</param>
    /// <param name="radius">Radius of the sphere or capsule shape. Must be greater than zero.</param>
    /// <param name="boxSize">Extents of the box shape. Every dimension must be greater than zero.</param>
    /// <param name="capsuleLength">Length of the capsule shape between the cap centers.</param>
    /// <param name="orientation">Orientation of the test shape. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="collisionMask">Which collision layers the query tests against. Null = Everything.</param>
    /// <param name="enabled">Skips the query and outputs an empty spread when false.</param>
    /// <returns>One entry per overlap with the collidable, penetration direction and length. Use Split to access the parts.</returns>
    [return: Pin(Name = "Output")]
    public Spread<SBepu.OverlapInfo> Update(
        SBepu.BepuSimulation? simulation,
        out Spread<SBepu.CollidableComponent> collidables,
        Vector3 position,
        SweepShape? shape,
        [DefaultValue(0.5f)] float radius,
        [Pin(Name = "Box Size"), DefaultValue("1.0, 1.0, 1.0")] Vector3 boxSize,
        [DefaultValue(1f)] float capsuleLength,
        Quaternion orientation,
        SBepu.CollisionMask? collisionMask,
        bool enabled = true)
    {
        if (!enabled || simulation is null)
        {
            collidables = _collidables = Spread<SBepu.CollidableComponent>.Empty;
            return _result = Spread<SBepu.OverlapInfo>.Empty;
        }

        var mask = collisionMask ?? SBepu.CollisionMask.Everything;
        _infoBuilder.Clear();
        var pose = new SDefinitions.RigidPose(position, orientation);
        switch (shape ?? SweepShape.Sphere)
        {
            case SweepShape.Sphere:
                simulation.Overlap(new Sphere(radius), pose, _infoBuilder, mask);
                break;
            case SweepShape.Box:
                simulation.Overlap(new Box(boxSize.X, boxSize.Y, boxSize.Z), pose, _infoBuilder, mask);
                break;
            case SweepShape.Capsule:
                simulation.Overlap(new Capsule(radius, capsuleLength), pose, _infoBuilder, mask);
                break;
        }

        if (_infoBuilder.Count == 0)
        {
            collidables = _collidables = Spread<SBepu.CollidableComponent>.Empty;
            return _result = Spread<SBepu.OverlapInfo>.Empty;
        }
        _collidableBuilder.Clear();
        for (var i = 0; i < _infoBuilder.Count; i++)
        {
            var collidable = _infoBuilder[i].Collidable;
            if (collidable is not null)
                _collidableBuilder.Add(collidable);
        }
        collidables = _collidables = _collidableBuilder.ToSpread();
        return _result = _infoBuilder.ToSpread();
    }
}

/// <summary>Shape used by sweep and overlap queries.</summary>
public enum SweepShape
{
    Sphere,
    Box,
    Capsule,
}
