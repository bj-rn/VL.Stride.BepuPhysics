using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Graphics;
using Stride.Graphics.Semantics;
using VL.Core;
using VL.Core.Import;
using VL.Lib.Basics.Resources;
using VL.Lib.Collections;
using SDefinitions = global::Stride.BepuPhysics.Definitions;
using SModel = global::Stride.Rendering.Model;

namespace VL.Stride.BepuPhysics.Colliders;

/// <summary>
/// Bakes convex hull data from a Stride Model's mesh vertices, for the ConvexHullCollider's
/// Hull pin. All vertices go into ONE hull: the physics shape is the model's convex envelope,
/// concave features are filled in (multi hull decomposition only exists in the Stride editor's
/// asset pipeline). Works with runtime and procedural models; imported assets need CPU
/// accessible mesh data, the same requirement MeshCollider has.
/// </summary>
[ProcessNode(Name = "HullFromModel")]
public class HullFromModelNode : IDisposable
{
    private readonly IResourceHandle<Game> _gameHandle = AppHost.Current.Services.GetGameHandle();
    private SModel? _lastModel;
    private SDefinitions.DecomposedHulls? _hulls;

    /// <param name="model">Stride Model whose vertices are baked into a convex hull. Outputs null while unconnected.</param>
    /// <param name="rebake">While true, re-extracts the model's mesh data. Connect a Bang after changing a runtime model's geometry.</param>
    /// <returns>Hull data for the ConvexHullCollider's Hull pin. Cached until the Model changes or Rebake fires.</returns>
    [return: Pin(Name = "Output")]
    public SDefinitions.DecomposedHulls? Update(SModel? model = null, bool rebake = false)
    {
        if (model is null)
        {
            _lastModel = null;
            return _hulls = null;
        }
        if (!ReferenceEquals(model, _lastModel) || rebake)
        {
            _lastModel = model;
            ExtractMeshBuffers(model, _gameHandle.Resource.Services, out var points, out var indices);
            var uintIndices = new uint[indices.Length];
            for (var i = 0; i < indices.Length; i++)
                uintIndices[i] = (uint)indices[i];
            _hulls = new SDefinitions.DecomposedHulls(new[]
            {
                new SDefinitions.DecomposedHulls.DecomposedMesh(new[]
                {
                    new SDefinitions.DecomposedHulls.Hull(points, uintIndices),
                }),
            });
        }
        return _hulls;
    }

    // Mirrors the engine's internal ShapeCacheSystem.ExtractMeshBuffers (MIT licensed), the
    // same path MeshCollider uses; the system itself is not public. Recheck at upgrade time.
    // Source: https://github.com/stride3d/stride/blob/releases/4.2.1.2487/sources/engine/Stride.BepuPhysics/Stride.BepuPhysics/Systems/ShapeCacheSystem.cs#L165
    internal static void ExtractMeshBuffers(SModel model, IServiceRegistry services, out Vector3[] vertices, out int[] indices)
    {
        int totalVertices = 0, totalIndices = 0;
        foreach (var mesh in model.Meshes)
        {
            totalVertices += mesh.Draw.VertexBuffers[0].Count;
            totalIndices += mesh.Draw.IndexBuffer.Count;
        }

        vertices = new Vector3[totalVertices];
        indices = new int[totalIndices];
        var verticesLeft = vertices.AsSpan();
        var indicesLeft = indices.AsSpan();

        foreach (var mesh in model.Meshes)
        {
            mesh.Draw.IndexBuffer.AsReadable(services, out var indexHelper, out var indexCount);
            mesh.Draw.VertexBuffers[0].AsReadable(services, out var vertexHelper, out var vertexCount);

            vertexHelper.Copy<PositionSemantic, Vector3>(verticesLeft[..vertexCount]);
            indexHelper.CopyTo(indicesLeft[..indexCount]);

            verticesLeft = verticesLeft[vertexCount..];
            indicesLeft = indicesLeft[indexCount..];
        }
    }

    public void Dispose() => _gameHandle.Dispose();
}

/// <summary>
/// Bakes convex hull data from a point cloud, for the ConvexHullCollider's Hull pin.
/// The physics shape is the convex envelope of the points, computed on attach.
/// </summary>
[ProcessNode(Name = "HullFromPoints")]
public class HullFromPointsNode
{
    private object? _lastPoints = new(); // sentinel so the first Update always bakes
    private SDefinitions.DecomposedHulls? _hulls;

    /// <param name="points">Points spanning the hull, in the collidable's local space. At least 4 points not lying in one plane are required, degenerate input fails on attach.</param>
    /// <returns>Hull data for the ConvexHullCollider's Hull pin. Cached until the spread changes.</returns>
    [return: Pin(Name = "Output")]
    public SDefinitions.DecomposedHulls? Update(Spread<Vector3>? points = null)
    {
        // Spread is immutable, a content change always means a new instance.
        if (!ReferenceEquals(_lastPoints, points))
        {
            _lastPoints = points;
            if (points is null || points.Count < 4)
            {
                _hulls = null;
            }
            else
            {
                var arr = new Vector3[points.Count];
                for (var i = 0; i < points.Count; i++)
                    arr[i] = points[i];
                // Indices are only used by the engine's visualization paths, the physics
                // hull is computed from the points alone.
                _hulls = new SDefinitions.DecomposedHulls(new[]
                {
                    new SDefinitions.DecomposedHulls.DecomposedMesh(new[]
                    {
                        new SDefinitions.DecomposedHulls.Hull(arr, Array.Empty<uint>()),
                    }),
                });
            }
        }
        return _hulls;
    }
}
