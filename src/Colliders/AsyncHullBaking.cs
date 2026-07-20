// Async variants of the hull baking nodes: the convex hull computation runs on a
// background task via the poll-based BackgroundComputation helper (latest-wins
// coalescing, sticky faults), so large point clouds don't stall the frame. The baked
// hull carries only the reduced hull vertices, which makes the engine's unavoidable
// main-thread hull build at collider attach trivial.
//
// Adoption rule: a completed bake is adopted only when it matches the current inputs.
// A bake that completed for older inputs is skipped (a fresh one starts immediately),
// so downstream collider reassigns happen once per settled input, and a hull from
// before a disconnect can never resurface after reconnecting.

using Stride.Core.Mathematics;
using Stride.Engine;
using VL.Core;
using VL.Core.Import;
using VL.Lib.Basics.Resources;
using VL.Lib.Collections;
using VL.Stride.BepuPhysics.Internal;
using SDefinitions = global::Stride.BepuPhysics.Definitions;
using SModel = global::Stride.Rendering.Model;

namespace VL.Stride.BepuPhysics.Colliders;

// Wraps the nullable bake outcome: a completed bake of degenerate content (for example
// coplanar points) must adopt as "output null", which the task result itself cannot
// express since BackgroundComputation requires a non-null result class.
internal sealed record HullBakeResult(SDefinitions.DecomposedHulls? Hulls);

/// <summary>
/// Like HullFromPoints, but reduces the points to their actual convex hull vertices on a
/// background thread. The output carries only the reduced points (typically dozens
/// instead of thousands), so the hull build the engine runs on the main thread when the
/// collider attaches becomes trivial. Unlike HullFromPoints the output carries triangle
/// data, so the ColliderShapes debug node can draw it.
/// </summary>
[ProcessNode(Name = "HullFromPoints (Async)")]
public class HullFromPointsAsyncNode
{
    private readonly BackgroundComputation<HullBakeResult> _computation = new();
    private object? _lastPoints = new(); // sentinel so the first Update always processes
    private Vector3[]? _points;          // snapshot for (re)starting the bake
    private int _version;
    private SDefinitions.DecomposedHulls? _hulls;

    /// <param name="inProgress">True while a hull is being baked in the background.</param>
    /// <param name="points">Points spanning the hull, in the collidable's local space. At least 4 distinct points not lying in one plane are required; degenerate input outputs null instead of failing at attach.</param>
    /// <returns>Reduced hull data for the ConvexHullCollider's Hull pin (the last completed bake while a new one is computed).</returns>
    [return: Pin(Name = "Output")]
    public SDefinitions.DecomposedHulls? Update(out bool inProgress, Spread<Vector3>? points = null)
    {
        // Spread is immutable, a content change always means a new instance.
        if (!ReferenceEquals(_lastPoints, points))
        {
            _lastPoints = points;
            _version++;
            if (points is null || points.Count < 4)
            {
                // Disconnected or trivially degenerate: output null immediately and
                // stop polling. A bake still in flight belongs to an older version and
                // its result must not resurface when the input returns.
                _points = null;
                _hulls = null;
            }
            else
            {
                var arr = new Vector3[points.Count];
                for (var i = 0; i < points.Count; i++)
                    arr[i] = points[i];
                _points = arr;
            }
        }

        if (_points is null)
        {
            inProgress = false;
            return _hulls;
        }

        bool adopted = _computation.Poll(_version, out var result, out bool needsStart, out inProgress);
        if (adopted && !needsStart)
            _hulls = result!.Hulls;
        if (needsStart)
        {
            var arr = _points;
            _computation.Start(_version, () => new HullBakeResult(ConvexHullBaker.Bake(arr)));
            inProgress = true;
        }
        return _hulls;
    }
}

/// <summary>
/// Bakes one convex hull per point group on a background thread, combined into a single
/// hull data output: connected to one ConvexHullCollider, the shapes act as one
/// collidable. Made for per-glyph physics on the Point Groups output of the async
/// Text3dMeshes nodes (VL.Stride.Text3d), where per-glyph hulls track text far better
/// than one hull around everything.
/// </summary>
[ProcessNode(Name = "HullsFromPointGroups (Async)")]
public class HullsFromPointGroupsAsyncNode
{
    private readonly BackgroundComputation<HullBakeResult> _computation = new();
    private object? _lastGroups = new(); // sentinel so the first Update always processes
    private Vector3[][]? _groups;        // snapshot for (re)starting the bake
    private int _version;
    private SDefinitions.DecomposedHulls? _hulls;

    /// <param name="inProgress">True while hulls are being baked in the background.</param>
    /// <param name="pointGroups">One group of points per hull, in the collidable's local space. Groups with fewer than 4 distinct points or all points in one plane are skipped; if no group forms a volume the output is null.</param>
    /// <returns>Hull data with one reduced hull per group, for the ConvexHullCollider's Hull pin (the last completed bake while a new one is computed).</returns>
    [return: Pin(Name = "Output")]
    public SDefinitions.DecomposedHulls? Update(out bool inProgress, Spread<Spread<Vector3>>? pointGroups = null)
    {
        // Outer-spread reference detection only: the Text3d point outputs rebuild the
        // whole nested spread per adoption, so an inner change always shows up as a new
        // outer instance. Hand-assembled inputs must follow the same convention.
        if (!ReferenceEquals(_lastGroups, pointGroups))
        {
            _lastGroups = pointGroups;
            _version++;
            if (pointGroups is null || pointGroups.Count == 0)
            {
                _groups = null;
                _hulls = null;
            }
            else
            {
                var groups = new Vector3[pointGroups.Count][];
                for (var g = 0; g < pointGroups.Count; g++)
                {
                    var group = pointGroups[g];
                    var arr = new Vector3[group.Count];
                    for (var i = 0; i < group.Count; i++)
                        arr[i] = group[i];
                    groups[g] = arr;
                }
                _groups = groups;
            }
        }

        if (_groups is null)
        {
            inProgress = false;
            return _hulls;
        }

        bool adopted = _computation.Poll(_version, out var result, out bool needsStart, out inProgress);
        if (adopted && !needsStart)
            _hulls = result!.Hulls;
        if (needsStart)
        {
            var groups = _groups;
            _computation.Start(_version, () => new HullBakeResult(ConvexHullBaker.BakeHulls(groups)));
            inProgress = true;
        }
        return _hulls;
    }
}

/// <summary>
/// Like HullFromModel, but reduces the mesh vertices to their actual convex hull on a
/// background thread. Only the hull computation runs in the background; reading the mesh
/// buffers back from the GPU needs the graphics device and stays on the main thread, on
/// every Model change and Rebake.
/// </summary>
[ProcessNode(Name = "HullFromModel (Async)")]
public class HullFromModelAsyncNode : IDisposable
{
    private readonly IResourceHandle<Game> _gameHandle = AppHost.Current.Services.GetGameHandle();
    private readonly BackgroundComputation<HullBakeResult> _computation = new();
    private SModel? _lastModel;
    private Vector3[]? _points;
    private int _version;
    private SDefinitions.DecomposedHulls? _hulls;

    /// <param name="inProgress">True while a hull is being baked in the background.</param>
    /// <param name="model">Stride Model whose vertices are baked into a convex hull. Outputs null while unconnected.</param>
    /// <param name="rebake">While true, re-extracts the model's mesh data. Connect a Bang after changing a runtime model's geometry.</param>
    /// <returns>Reduced hull data for the ConvexHullCollider's Hull pin (the last completed bake while a new one is computed).</returns>
    [return: Pin(Name = "Output")]
    public SDefinitions.DecomposedHulls? Update(out bool inProgress, SModel? model = null, bool rebake = false)
    {
        if (!ReferenceEquals(model, _lastModel) || rebake)
        {
            _lastModel = model;
            _version++;
            if (model is null)
            {
                _points = null;
                _hulls = null;
            }
            else
            {
                // Main-thread GPU readback, shared with the sync node; only the hull
                // math afterwards runs in the background.
                HullFromModelNode.ExtractMeshBuffers(model, _gameHandle.Resource.Services, out var vertices, out _);
                _points = vertices;
            }
        }

        if (_points is null)
        {
            inProgress = false;
            return _hulls;
        }

        bool adopted = _computation.Poll(_version, out var result, out bool needsStart, out inProgress);
        if (adopted && !needsStart)
            _hulls = result!.Hulls;
        if (needsStart)
        {
            var arr = _points;
            _computation.Start(_version, () => new HullBakeResult(ConvexHullBaker.Bake(arr)));
            inProgress = true;
        }
        return _hulls;
    }

    public void Dispose() => _gameHandle.Dispose();
}
