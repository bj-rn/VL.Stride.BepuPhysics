using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using Stride.Core.Mathematics;
using SDefinitions = global::Stride.BepuPhysics.Definitions;

namespace VL.Stride.BepuPhysics.Internal;

/// <summary>
/// Pure hull math for the async hull baking nodes: reduces a point cloud to its convex
/// hull vertices via BepuPhysics' public ConvexHullHelper, so the engine's unavoidable
/// attach-time hull build runs over ~dozens of points instead of thousands.
///
/// THREADING: BufferPool is NOT thread-safe and the simulation's pool must never be
/// used here. Each invocation creates its own pool and clears it in finally; the pool
/// never escapes the call. The callers' latest-wins BackgroundComputation guarantees at
/// most one in-flight task per node, and abandoned tasks clear their own pool.
/// </summary>
internal static class ConvexHullBaker
{
    /// <summary>
    /// Reduces the points to their convex hull. Returns null for degenerate input that
    /// cannot form a volume: fewer than 4 distinct points, or a hull with fewer than
    /// 4 faces (ComputeHull does not throw on coplanar/collinear input; it returns a
    /// flat 2-face "hull" for coplanar points and an empty one for collinear points,
    /// verified against BepuPhysics 2.5.0-beta.25).
    /// </summary>
    internal static SDefinitions.DecomposedHulls.Hull? BakeHull(Vector3[] points)
    {
        var distinct = Deduplicate(points);
        if (distinct.Length < 4)
            return null;

        // Stride and System.Numerics Vector3 are both three sequential floats, but an
        // explicit copy keeps us independent of layout assumptions.
        var numericsPoints = new System.Numerics.Vector3[distinct.Length];
        for (var i = 0; i < distinct.Length; i++)
            numericsPoints[i] = new System.Numerics.Vector3(distinct[i].X, distinct[i].Y, distinct[i].Z);

        int[] vertexMapping;
        int[] faceStarts;
        int[] faceVertexIndices;
        var pool = new BufferPool();
        try
        {
            ConvexHullHelper.ComputeHull(numericsPoints.AsSpan(), pool, out var hullData);
            vertexMapping = ToArray(hullData.OriginalVertexMapping);
            faceStarts = ToArray(hullData.FaceStartIndices);
            faceVertexIndices = ToArray(hullData.FaceVertexIndices);
        }
        finally
        {
            // Returns ALL pool memory (including HullData's buffers), nothing leaks.
            pool.Clear();
        }

        // A hull enclosing a volume has at least 4 vertices AND 4 faces; coplanar input
        // yields 2 faces and would fail in the engine's attach-time ConvexHull build.
        if (vertexMapping.Length < 4 || faceStarts.Length < 4)
            return null;

        var reduced = new Vector3[vertexMapping.Length];
        for (var i = 0; i < vertexMapping.Length; i++)
            reduced[i] = distinct[vertexMapping[i]];

        // Fan-triangulate the (possibly n-gon) faces. FaceVertexIndices already index
        // the reduced set (verified against 2.5.0-beta.25). The physics hull is built
        // from the points alone; the indices make the hull visible in the
        // ColliderShapes debug node.
        var triangleIndexCount = 0;
        for (var f = 0; f < faceStarts.Length; f++)
        {
            var start = faceStarts[f];
            var end = f + 1 < faceStarts.Length ? faceStarts[f + 1] : faceVertexIndices.Length;
            triangleIndexCount += (end - start - 2) * 3;
        }

        var indices = new uint[triangleIndexCount];
        var w = 0;
        for (var f = 0; f < faceStarts.Length; f++)
        {
            var start = faceStarts[f];
            var end = f + 1 < faceStarts.Length ? faceStarts[f + 1] : faceVertexIndices.Length;
            for (var i = start + 1; i < end - 1; i++)
            {
                indices[w++] = (uint)faceVertexIndices[start];
                indices[w++] = (uint)faceVertexIndices[i];
                indices[w++] = (uint)faceVertexIndices[i + 1];
            }
        }

        return new SDefinitions.DecomposedHulls.Hull(reduced, indices);
    }

    /// <summary>
    /// One hull per point group (per-glyph pipeline); degenerate groups are skipped.
    /// Returns null when no group yields a hull.
    /// </summary>
    internal static SDefinitions.DecomposedHulls? BakeHulls(Vector3[][] groups)
    {
        var hulls = new List<SDefinitions.DecomposedHulls.Hull>(groups.Length);
        foreach (var group in groups)
        {
            if (BakeHull(group) is { } hull)
                hulls.Add(hull);
        }
        if (hulls.Count == 0)
            return null;
        return new SDefinitions.DecomposedHulls(new[]
        {
            new SDefinitions.DecomposedHulls.DecomposedMesh(hulls.ToArray()),
        });
    }

    /// <summary>Single-hull wrapper around <see cref="BakeHull"/>.</summary>
    internal static SDefinitions.DecomposedHulls? Bake(Vector3[] points)
    {
        if (BakeHull(points) is not { } hull)
            return null;
        return new SDefinitions.DecomposedHulls(new[]
        {
            new SDefinitions.DecomposedHulls.DecomposedMesh(new[] { hull }),
        });
    }

    /// <summary>Exact-equality dedup preserving first-seen order.</summary>
    internal static Vector3[] Deduplicate(ReadOnlySpan<Vector3> points)
    {
        var seen = new HashSet<Vector3>(points.Length);
        var distinct = new List<Vector3>(points.Length);
        foreach (var p in points)
        {
            if (seen.Add(p))
                distinct.Add(p);
        }
        return distinct.ToArray();
    }

    private static int[] ToArray(Buffer<int> buffer)
    {
        var result = new int[buffer.Length];
        for (var i = 0; i < buffer.Length; i++)
            result[i] = buffer[i];
        return result;
    }
}
