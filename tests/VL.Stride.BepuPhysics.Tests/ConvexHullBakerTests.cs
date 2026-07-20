// Headless tests for the pure hull math behind the async hull baking nodes.
// Degenerate-input expectations reflect observed ComputeHull behavior on BepuPhysics
// 2.5.0-beta.25: coplanar input yields a flat 2-face hull, collinear input an empty
// one — neither throws; the baker rejects both structurally (< 4 vertices or faces).

using NUnit.Framework;
using Stride.Core.Mathematics;
using VL.Stride.BepuPhysics.Internal;

namespace VL.Stride.BepuPhysics.Tests;

[TestFixture]
public class ConvexHullBakerTests
{
    private static Vector3[] CubeCorners()
    {
        var corners = new Vector3[8];
        var k = 0;
        for (var x = 0; x <= 1; x++)
            for (var y = 0; y <= 1; y++)
                for (var z = 0; z <= 1; z++)
                    corners[k++] = new Vector3(x, y, z);
        return corners;
    }

    private static Vector3[] CubeWithInteriorPoints(int interiorCount, int seed = 42)
    {
        var corners = CubeCorners();
        var points = new Vector3[corners.Length + interiorCount];
        corners.CopyTo(points, 0);
        var rng = new Random(seed);
        for (var i = 0; i < interiorCount; i++)
            points[corners.Length + i] = new Vector3(
                0.05f + 0.9f * (float)rng.NextDouble(),
                0.05f + 0.9f * (float)rng.NextDouble(),
                0.05f + 0.9f * (float)rng.NextDouble());
        return points;
    }

    [Test]
    public void CubeWithInteriorPointsReducesToCorners()
    {
        var input = CubeWithInteriorPoints(500);
        var hull = ConvexHullBaker.BakeHull(input);

        Assert.That(hull, Is.Not.Null);
        Assert.That(hull!.Points.ToArray(), Is.EquivalentTo(CubeCorners()));
    }

    [Test]
    public void ReducedPointsAreSubsetOfInput()
    {
        var input = CubeWithInteriorPoints(100);
        var hull = ConvexHullBaker.BakeHull(input)!;

        var inputSet = new HashSet<Vector3>(input);
        foreach (var p in hull.Points)
            Assert.That(inputSet, Does.Contain(p));
    }

    [Test]
    public void HullContainsAllInputPoints()
    {
        var input = CubeWithInteriorPoints(200, seed: 7);
        var hull = ConvexHullBaker.BakeHull(input)!;
        var points = hull.Points.ToArray();
        var indices = hull.Indices.ToArray();

        // Hull centroid is strictly inside a convex volume: orient each triangle's
        // normal away from it, then every input point must lie on or behind each face.
        var centroid = Vector3.Zero;
        foreach (var p in points)
            centroid += p;
        centroid /= points.Length;

        const float epsilon = 1e-4f;
        for (var t = 0; t < indices.Length; t += 3)
        {
            var a = points[indices[t]];
            var b = points[indices[t + 1]];
            var c = points[indices[t + 2]];
            var normal = Vector3.Cross(b - a, c - a);
            if (Vector3.Dot(normal, centroid - a) > 0)
                normal = -normal;
            normal.Normalize();

            foreach (var p in input)
                Assert.That(Vector3.Dot(normal, p - a), Is.LessThanOrEqualTo(epsilon),
                    $"point {p} lies outside triangle {t / 3}");
        }
    }

    [Test]
    public void IndicesAreValidTriangles()
    {
        var hull = ConvexHullBaker.BakeHull(CubeWithInteriorPoints(50))!;

        Assert.That(hull.Indices.Length, Is.GreaterThanOrEqualTo(12)); // >= 4 faces, fan-triangulated
        Assert.That(hull.Indices.Length % 3, Is.Zero);
        foreach (var index in hull.Indices)
            Assert.That(index, Is.LessThan((uint)hull.Points.Length));

        // Every hull vertex is referenced by at least one triangle
        var referenced = new HashSet<uint>(hull.Indices.ToArray());
        Assert.That(referenced.Count, Is.EqualTo(hull.Points.Length));
    }

    [Test]
    public void BakeHullsProducesOneHullPerGroupAndSkipsShortGroups()
    {
        var cubeA = CubeCorners();
        var cubeB = CubeCorners().Select(p => p + new Vector3(5, 0, 0)).ToArray();
        var shortGroup = new[] { new Vector3(9, 9, 9), new Vector3(10, 9, 9), new Vector3(9, 10, 9) };

        var hulls = ConvexHullBaker.BakeHulls(new[] { cubeA, shortGroup, cubeB });

        Assert.That(hulls, Is.Not.Null);
        Assert.That(hulls!.Meshes.Length, Is.EqualTo(1));
        Assert.That(hulls.Meshes[0].Hulls.Length, Is.EqualTo(2));
        Assert.That(hulls.Meshes[0].Hulls[0].Points.ToArray(), Is.EquivalentTo(cubeA));
        Assert.That(hulls.Meshes[0].Hulls[1].Points.ToArray(), Is.EquivalentTo(cubeB));
    }

    [Test]
    public void DegenerateInputsYieldNull()
    {
        // Fewer than 4 distinct points
        Assert.That(ConvexHullBaker.BakeHull(Array.Empty<Vector3>()), Is.Null);
        Assert.That(ConvexHullBaker.BakeHull(new[]
        {
            new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0),
        }), Is.Null);

        // 4 points collapsing to 3 distinct ones
        Assert.That(ConvexHullBaker.BakeHull(new[]
        {
            new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 0),
        }), Is.Null);

        // Collinear
        Assert.That(ConvexHullBaker.BakeHull(new[]
        {
            new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(2, 0, 0), new Vector3(3, 0, 0),
        }), Is.Null);

        Assert.That(ConvexHullBaker.Bake(Array.Empty<Vector3>()), Is.Null);
    }

    [Test]
    public void CoplanarInputYieldsNull()
    {
        // ComputeHull returns a flat 2-face hull for coplanar points (observed on
        // 2.5.0-beta.25); the baker rejects it so the engine's attach-time hull build
        // never sees a volume-less shape. This is the extrudeAmount = 0 text case.
        var grid = new List<Vector3>();
        for (var x = 0; x < 4; x++)
            for (var y = 0; y < 4; y++)
                grid.Add(new Vector3(x, y, 0));

        Assert.That(ConvexHullBaker.BakeHull(grid.ToArray()), Is.Null);
    }

    [Test]
    public void BakeHullsWithOnlyDegenerateGroupsYieldsNull()
    {
        var flat = new[] { new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(1, 1, 0) };
        Assert.That(ConvexHullBaker.BakeHulls(new[] { flat, Array.Empty<Vector3>() }), Is.Null);
    }

    [Test]
    public void DeduplicateRemovesExactDuplicatesAndPreservesOrder()
    {
        var points = new[]
        {
            new Vector3(1, 2, 3), new Vector3(4, 5, 6), new Vector3(1, 2, 3), new Vector3(7, 8, 9),
            new Vector3(4, 5, 6),
        };

        var distinct = ConvexHullBaker.Deduplicate(points);

        Assert.That(distinct, Is.EqualTo(new[]
        {
            new Vector3(1, 2, 3), new Vector3(4, 5, 6), new Vector3(7, 8, 9),
        }));
    }
}
