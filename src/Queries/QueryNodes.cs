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
    /// <summary>Casts a ray and reports the closest hit.</summary>
    /// <param name="simulation">The simulation to query - from a SimulationSettings or GetSimulation node.</param>
    /// <param name="hit">The closest hit: point, normal, distance and the collidable that was hit.</param>
    /// <param name="didHit">True when the ray hit anything within Max Distance.</param>
    /// <param name="origin">Ray start position in world space.</param>
    /// <param name="direction">Ray direction in world space (does not need to be normalized).</param>
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
        didHit = simulation.RayCast(origin, direction, maxDistance, out hit, collisionMask ?? SBepu.CollisionMask.Everything);
    }
}

/// <summary>
/// Casts a ray and reports all hits along it, sorted by distance.
/// </summary>
[ProcessNode(Name = "RayCastPenetrating")]
public class RayCastPenetratingNode
{
    private readonly List<SBepu.HitInfo> _buffer = new();
    private readonly SpreadBuilder<SBepu.HitInfo> _builder = new();
    private Spread<SBepu.HitInfo> _result = Spread<SBepu.HitInfo>.Empty;

    /// <param name="simulation">The simulation to query — from a SimulationSettings or GetSimulation node.</param>
    /// <param name="origin">Ray start position in world space.</param>
    /// <param name="direction">Ray direction in world space (does not need to be normalized).</param>
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

        _buffer.Clear();
        simulation.RayCastPenetrating(origin, direction, maxDistance, _buffer, collisionMask ?? SBepu.CollisionMask.Everything);
        return _result = ToSpread(_buffer, _builder, _result);
    }

    internal static Spread<SBepu.HitInfo> ToSpread(List<SBepu.HitInfo> buffer, SpreadBuilder<SBepu.HitInfo> builder, Spread<SBepu.HitInfo> last)
    {
        if (buffer.Count == 0)
            return Spread<SBepu.HitInfo>.Empty;
        builder.Clear();
        foreach (var item in buffer)
            builder.Add(item);
        return builder.ToSpread();
    }
}

/// <summary>
/// Sweeps a shape along a direction and reports all hits.
/// </summary>
[ProcessNode(Name = "SweepCast")]
public class SweepCastNode
{
    private readonly List<SBepu.HitInfo> _buffer = new();
    private readonly SpreadBuilder<SBepu.HitInfo> _builder = new();
    private Spread<SBepu.HitInfo> _result = Spread<SBepu.HitInfo>.Empty;

    /// <param name="simulation">The simulation to query — from a SimulationSettings or GetSimulation node.</param>
    /// <param name="origin">Ray start position in world space.</param>
    /// <param name="direction">Ray direction in world space (does not need to be normalized).</param>
    /// <param name="maxDistance">Maximum travel distance of the query.</param>
    /// <param name="shape">Shape used by the query. Null = Sphere.</param>
    /// <param name="radius">Radius of the sphere or capsule shape.</param>
    /// <param name="boxSize">Extents of the box shape.</param>
    /// <param name="capsuleLength">Length of the capsule shape between the cap centers.</param>
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
        [Pin(Name = "Box Size"), DefaultValue(typeof(Vector3), "1.0, 1.0, 1.0")] Vector3 boxSize,
        [DefaultValue(1f)] float capsuleLength,
        SBepu.CollisionMask? collisionMask,
        bool enabled = true)
    {
        if (!enabled || simulation is null || direction == Vector3.Zero)
            return _result = Spread<SBepu.HitInfo>.Empty;

        if (boxSize == default)
            boxSize = Vector3.One;

        var mask = collisionMask ?? SBepu.CollisionMask.Everything;
        _buffer.Clear();
        var pose = new SDefinitions.RigidPose(origin, Quaternion.Identity);
        var velocity = new SDefinitions.BodyVelocity(direction, Vector3.Zero);
        switch (shape ?? SweepShape.Sphere)
        {
            case SweepShape.Sphere:
                simulation.SweepCastPenetrating(new Sphere(radius), pose, velocity, maxDistance, _buffer, mask);
                break;
            case SweepShape.Box:
                simulation.SweepCastPenetrating(new Box(boxSize.X, boxSize.Y, boxSize.Z), pose, velocity, maxDistance, _buffer, mask);
                break;
            case SweepShape.Capsule:
                simulation.SweepCastPenetrating(new Capsule(radius, capsuleLength), pose, velocity, maxDistance, _buffer, mask);
                break;
        }
        return _result = RayCastPenetratingNode.ToSpread(_buffer, _builder, _result);
    }
}

/// <summary>
/// Reports all collidables overlapping a shape placed at a position.
/// </summary>
[ProcessNode(Name = "Overlap")]
public class OverlapNode
{
    private readonly List<SBepu.OverlapInfo> _buffer = new();
    private readonly SpreadBuilder<SBepu.CollidableComponent> _builder = new();
    private Spread<SBepu.CollidableComponent> _result = Spread<SBepu.CollidableComponent>.Empty;

    /// <param name="simulation">The simulation to query — from a SimulationSettings or GetSimulation node.</param>
    /// <param name="position">Center of the test shape in world space.</param>
    /// <param name="shape">Shape used by the query. Null = Sphere.</param>
    /// <param name="radius">Radius of the sphere or capsule shape.</param>
    /// <param name="boxSize">Extents of the box shape.</param>
    /// <param name="capsuleLength">Length of the capsule shape between the cap centers.</param>
    /// <param name="collisionMask">Which collision layers the query tests against. Null = Everything.</param>
    /// <param name="enabled">Skips the query and outputs an empty spread when false.</param>
    [return: Pin(Name = "Output")]
    public Spread<SBepu.CollidableComponent> Update(
        SBepu.BepuSimulation? simulation,
        Vector3 position,
        SweepShape? shape,
        [DefaultValue(0.5f)] float radius,
        [Pin(Name = "Box Size"), DefaultValue(typeof(Vector3), "1.0, 1.0, 1.0")] Vector3 boxSize,
        [DefaultValue(1f)] float capsuleLength,
        SBepu.CollisionMask? collisionMask,
        bool enabled = true)
    {
        if (!enabled || simulation is null)
            return _result = Spread<SBepu.CollidableComponent>.Empty;

        if (boxSize == default)
            boxSize = Vector3.One;

        var mask = collisionMask ?? SBepu.CollisionMask.Everything;
        _buffer.Clear();
        var pose = new SDefinitions.RigidPose(position, Quaternion.Identity);
        switch (shape ?? SweepShape.Sphere)
        {
            case SweepShape.Sphere:
                simulation.Overlap(new Sphere(radius), pose, _buffer, mask);
                break;
            case SweepShape.Box:
                simulation.Overlap(new Box(boxSize.X, boxSize.Y, boxSize.Z), pose, _buffer, mask);
                break;
            case SweepShape.Capsule:
                simulation.Overlap(new Capsule(radius, capsuleLength), pose, _buffer, mask);
                break;
        }

        if (_buffer.Count == 0)
            return _result = Spread<SBepu.CollidableComponent>.Empty;
        _builder.Clear();
        foreach (var info in _buffer)
        {
            if (info.Collidable is not null)
                _builder.Add(info.Collidable);
        }
        return _result = _builder.ToSpread();
    }
}

/// <summary>Shape used by sweep and overlap queries.</summary>
public enum SweepShape
{
    Sphere,
    Box,
    Capsule,
}
