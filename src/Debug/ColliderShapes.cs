using Stride.Core.Mathematics;
using Stride.Extensions;
using Stride.Engine;
using Stride.Graphics;
using Stride.Graphics.GeometricPrimitives;
using Stride.Rendering;
using Stride.Rendering.Materials;
using Stride.Rendering.Materials.ComputeColors;
using VL.Core;
using VL.Core.Import;
using VL.Lib.Basics.Resources;
using VL.Stride.BepuPhysics.Internal;
using VL.Stride.Rendering;
using SBepu = global::Stride.BepuPhysics;
using SColliders = global::Stride.BepuPhysics.Definitions.Colliders;
using SModel = global::Stride.Rendering.Model;
using Buffer = Stride.Graphics.Buffer;

namespace VL.Stride.BepuPhysics.Debug;

/// <summary>
/// Debug view of the physics world: builds an entity with one child per collider shape in
/// the simulation, rendered through the normal Stride pipeline with a translucent material
/// and wireframe lines (vvvv's Wireframe render feature). Connect the Output directly to the
/// RootScene's Children input; a transformed parent would offset the shapes. Bodies follow
/// their raw physics poses, revealing discrepancies between the simulation and the rendered
/// entities; statics follow their entity transforms. Convex hulls baked with HullFromPoints
/// carry no triangle data and are skipped, hulls from HullFromModel are drawn.
/// For very large simulations the Instanced mode collapses all shapes sharing a mesh into
/// one draw call each, at the cost of the wireframe lines: vvvv's wireframe render feature
/// does not support instancing, so only the shape fill remains.
/// </summary>
[ProcessNode(Name = "ColliderShapes")]
public class ColliderShapesNode : IDisposable
{
    private sealed class ShapeSlot
    {
        public required Entity Entity;
        public required ModelComponent ModelComponent;
        public required WireframeComponent Wireframe;
        public SModel? CurrentModel;
        public Material? CurrentMaterial;
    }

    // One entity per distinct mesh in instanced mode, all its shapes are drawn in a single
    // instanced draw call. The world matrices array is pooled and grows on demand.
    private sealed class InstanceGroup
    {
        public required ModelComponent ModelComponent;
        public required InstancingUserArray Instances;
        public Material? CurrentMaterial;
        public Matrix[] Matrices = new Matrix[64];
        public int Count;
    }

    private readonly IResourceHandle<Game> _gameHandle = AppHost.Current.Services.GetGameHandle();
    private readonly Entity _root = new("Collider Shapes");
    private readonly List<ShapeSlot> _slots = new();
    private readonly Dictionary<SModel, InstanceGroup> _groups = new();
    // Typed caches: value tuple keys stay unboxed, the object keyed cache only holds
    // reference keys (DecomposedHulls and Model instances), so no per frame boxing occurs.
    private SModel? _boxModel;
    private SModel? _sphereModel;
    private SModel? _cylinderModel;
    private readonly Dictionary<(float Radius, float Length), SModel> _capsuleModels = new();
    private readonly Dictionary<(Vector3 A, Vector3 B, Vector3 C), SModel> _triangleModels = new();
    private readonly Dictionary<object, SModel?> _dataModels = new();
    private readonly List<IDisposable> _resources = new();
    private Material? _defaultMaterial;

    public ColliderShapesNode()
    {
        BepuSettingsBootstrap.EnsureConfigured();
    }

    /// <param name="simulation">The simulation whose colliders are shown, from a SimulationSettings or GetSimulation node.</param>
    /// <param name="visible">Shows or hides all collider shapes.</param>
    /// <param name="material">Material for the shape surfaces. Null = a translucent default. Connect an opaque material if the wireframe lines should always win over the fill.</param>
    /// <param name="lineWidth">Width of the wireframe lines. Ignored in Instanced mode.</param>
    /// <param name="lineColor">Color of the wireframe lines. Null = white. Ignored in Instanced mode.</param>
    /// <param name="instanced">Draws all shapes sharing a mesh in one instanced draw call, for very large simulations. CAVEAT: wireframe lines are not drawn in this mode, vvvv's wireframe render feature does not support instancing, only the shape fill remains.</param>
    /// <returns>The debug entity, connect directly to the RootScene's Children input.</returns>
    [return: Pin(Name = "Output")]
    public Entity Update(
        SBepu.BepuSimulation? simulation,
        bool visible = true,
        Material? material = null,
        float lineWidth = 1f,
        Color4? lineColor = null,
        bool instanced = false)
    {
        var device = _gameHandle.Resource.GraphicsDevice;
        var effectiveMaterial = material ?? GetDefaultMaterial(device);
        var effectiveLineColor = lineColor ?? new Color4(1f, 1f, 1f, 1f);

        foreach (var group in _groups.Values)
            group.Count = 0;

        var used = 0;
        if (simulation is not null && visible)
        {
            var bodies = simulation.Simulation.Bodies;
            for (var setIndex = 0; setIndex < bodies.Sets.Length; setIndex++)
            {
                ref var set = ref bodies.Sets[setIndex];
                if (!set.Allocated)
                    continue;
                for (var i = 0; i < set.Count; i++)
                {
                    var component = simulation.GetComponent(set.IndexToHandle[i]);
                    if (component is not null)
                        used = SyncCollidable(device, component, used, instanced, effectiveMaterial, lineWidth, effectiveLineColor);
                }
            }
            var statics = simulation.Simulation.Statics;
            for (var i = 0; i < statics.Count; i++)
            {
                var component = simulation.GetComponent(statics.IndexToHandle[i]);
                if (component is not null)
                    used = SyncCollidable(device, component, used, instanced, effectiveMaterial, lineWidth, effectiveLineColor);
            }
        }

        // Disable leftover slots instead of destroying them, they are reused when shapes return.
        // In instanced mode nothing marks a slot used, so this disables all of them.
        for (var i = used; i < _slots.Count; i++)
        {
            _slots[i].ModelComponent.Enabled = false;
            _slots[i].Wireframe.Enabled = false;
        }

        // Push the collected matrices; groups that received no shapes this frame (including
        // all of them when Instanced is off) are disabled, not destroyed.
        foreach (var group in _groups.Values)
        {
            if (group.Count == 0)
            {
                group.ModelComponent.Enabled = false;
                continue;
            }
            group.ModelComponent.Enabled = true;
            if (!ReferenceEquals(group.CurrentMaterial, effectiveMaterial))
            {
                group.CurrentMaterial = effectiveMaterial;
                group.ModelComponent.Materials[0] = effectiveMaterial;
            }
            group.Instances.UpdateWorldMatrices(group.Matrices, group.Count);
        }

        return _root;
    }

    private int SyncCollidable(GraphicsDevice device, SBepu.CollidableComponent collidable, int slotIndex,
        bool instanced, Material material, float lineWidth, Color4 lineColor)
    {
        GetCollidablePose(collidable, out var worldPosition, out var worldRotation);

        switch (collidable.Collider)
        {
            case SColliders.CompoundCollider compound:
                foreach (var shape in compound.Colliders)
                {
                    if (shape is null)
                        continue;
                    var model = ModelForShape(device, shape, out var scale);
                    if (model is null)
                        continue;
                    var position = worldPosition + Vector3.Transform(shape.PositionLocal, worldRotation);
                    var rotation = worldRotation * shape.RotationLocal;
                    slotIndex = Emit(slotIndex, instanced, model, position, rotation, scale, material, lineWidth, lineColor);
                }
                break;
            case SColliders.MeshCollider { Model: not null } meshCollider:
            {
                var model = ModelForMesh(device, meshCollider.Model);
                if (model is not null)
                {
                    var scale = SColliders.MeshCollider.ComputeMeshScale(collidable);
                    slotIndex = Emit(slotIndex, instanced, model, worldPosition, worldRotation, scale, material, lineWidth, lineColor);
                }
                break;
            }
        }
        return slotIndex;
    }

    private int Emit(int slotIndex, bool instanced, SModel model, Vector3 position, Quaternion rotation, Vector3 scale,
        Material material, float lineWidth, Color4 lineColor)
    {
        if (instanced)
        {
            AddInstance(model, position, rotation, scale);
            return slotIndex;
        }
        Apply(slotIndex, model, position, rotation, scale, material, lineWidth, lineColor);
        return slotIndex + 1;
    }

    private void AddInstance(SModel model, Vector3 position, Quaternion rotation, Vector3 scale)
    {
        if (!_groups.TryGetValue(model, out var group))
        {
            var entity = new Entity("Collider Shapes Instanced");
            var modelComponent = new ModelComponent { Model = model };
            // ModelTransformUsage stays at Ignore, the matrices are absolute world transforms
            // (the node is documented to sit untransformed under the RootScene).
            var instances = new InstancingUserArray();
            entity.Add(modelComponent);
            entity.Add(new InstancingComponent { Type = instances });
            _root.AddChild(entity);
            group = new InstanceGroup { ModelComponent = modelComponent, Instances = instances };
            _groups.Add(model, group);
        }
        if (group.Count == group.Matrices.Length)
            Array.Resize(ref group.Matrices, group.Matrices.Length * 2);
        Matrix.Transformation(ref scale, ref rotation, ref position, out group.Matrices[group.Count++]);
    }

    private static void GetCollidablePose(SBepu.CollidableComponent collidable, out Vector3 position, out Quaternion rotation)
    {
        if (collidable is SBepu.BodyComponent body && body.Simulation is not null)
        {
            // The physics pose is centered at the center of mass, the shapes are relative to
            // the body ORIGIN, shift back like the engine's own debug renderer does.
            rotation = body.Orientation;
            position = body.Position - Vector3.Transform(body.CenterOfMass, rotation);
        }
        else
        {
            // Statics (and unattached bodies) follow their entity's transform.
            var world = collidable.Entity?.Transform.WorldMatrix ?? Matrix.Identity;
            world.Decompose(out _, out rotation, out position);
        }
    }

    private void Apply(int slotIndex, SModel model, Vector3 position, Quaternion rotation, Vector3 scale,
        Material material, float lineWidth, Color4 lineColor)
    {
        while (_slots.Count <= slotIndex)
        {
            var entity = new Entity("Collider Shape");
            var modelComponent = new ModelComponent();
            var wireframe = new WireframeComponent();
            entity.Add(modelComponent);
            entity.Add(wireframe);
            _root.AddChild(entity);
            _slots.Add(new ShapeSlot { Entity = entity, ModelComponent = modelComponent, Wireframe = wireframe });
        }

        var slot = _slots[slotIndex];
        if (!ReferenceEquals(slot.CurrentModel, model))
        {
            slot.CurrentModel = model;
            slot.ModelComponent.Model = model;
        }
        if (!ReferenceEquals(slot.CurrentMaterial, material))
        {
            slot.CurrentMaterial = material;
            slot.ModelComponent.Materials[0] = material;
        }
        slot.ModelComponent.Enabled = true;
        slot.Wireframe.Enabled = true;
        slot.Wireframe.LineWidth = lineWidth;
        slot.Wireframe.Color = lineColor;

        var transform = slot.Entity.Transform;
        transform.Position = position;
        transform.Rotation = rotation;
        transform.Scale = scale;
    }

    private SModel? ModelForShape(GraphicsDevice device, SColliders.ColliderBase shape, out Vector3 scale)
    {
        switch (shape)
        {
            case SColliders.BoxCollider box:
                scale = box.Size;
                return _boxModel ??= BuildPrimitiveModel(GeometricPrimitive.Cube.New(device, 1f));
            case SColliders.SphereCollider sphere:
                scale = new Vector3(sphere.Radius);
                return _sphereModel ??= BuildPrimitiveModel(GeometricPrimitive.Sphere.New(device, 1f, 16));
            case SColliders.CylinderCollider cylinder:
                scale = new Vector3(cylinder.Radius, cylinder.Length, cylinder.Radius);
                return _cylinderModel ??= BuildPrimitiveModel(GeometricPrimitive.Cylinder.New(device, 1f, 1f, 16));
            case SColliders.CapsuleCollider capsule:
            {
                // A capsule cannot be built by scaling a unit capsule (the caps would distort),
                // cache one mesh per parameter pair instead. The key is rounded so animated
                // dimensions coalesce into a bounded number of meshes (a slightly stepped
                // debug shape instead of a new GPU mesh per frame).
                scale = Vector3.One;
                var key = (MathF.Round(capsule.Radius, 2), MathF.Round(capsule.Length, 2));
                if (!_capsuleModels.TryGetValue(key, out var model))
                {
                    model = BuildPrimitiveModel(GeometricPrimitive.Capsule.New(device, key.Item2, key.Item1, 8));
                    _capsuleModels[key] = model;
                }
                return model;
            }
            case SColliders.TriangleCollider triangle:
            {
                scale = Vector3.One;
                var key = (Round(triangle.A), Round(triangle.B), Round(triangle.C));
                if (!_triangleModels.TryGetValue(key, out var model))
                {
                    model = BuildTriangleModel(device, key.Item1, key.Item2, key.Item3);
                    _triangleModels[key] = model;
                }
                return model;
            }
            case SColliders.ConvexHullCollider { Hull: not null } hull:
            {
                scale = Vector3.One;
                if (!_dataModels.TryGetValue(hull.Hull, out var model))
                {
                    model = BuildHullModel(device, hull.Hull);
                    _dataModels[hull.Hull] = model;
                }
                return model;
            }
            default:
                scale = Vector3.One;
                return null;
        }
    }

    private static Vector3 Round(Vector3 value)
        => new(MathF.Round(value.X, 2), MathF.Round(value.Y, 2), MathF.Round(value.Z, 2));

    private SModel? ModelForMesh(GraphicsDevice device, SModel colliderModel)
    {
        if (_dataModels.TryGetValue(colliderModel, out var model))
            return model;
        Colliders.HullFromModelNode.ExtractMeshBuffers(colliderModel, _gameHandle.Resource.Services, out var positions, out var indices);
        model = indices.Length >= 3 ? BuildModel(device, positions, indices) : null;
        _dataModels[colliderModel] = model;
        return model;
    }

    private SModel BuildPrimitiveModel(GeometricPrimitive primitive)
    {
        _resources.Add(primitive);
        return new SModel { new Mesh { Draw = primitive.ToMeshDraw() } };
    }

    private SModel BuildTriangleModel(GraphicsDevice device, Vector3 a, Vector3 b, Vector3 c)
    {
        // Both windings so the triangle is visible from either side.
        return BuildModel(device, new[] { a, b, c }, new[] { 0, 1, 2, 0, 2, 1 });
    }

    private SModel? BuildHullModel(GraphicsDevice device, global::Stride.BepuPhysics.Definitions.DecomposedHulls hulls)
    {
        var totalVertices = 0;
        var totalIndices = 0;
        foreach (var mesh in hulls.Meshes)
        {
            foreach (var hull in mesh.Hulls)
            {
                totalVertices += hull.Points.Length;
                totalIndices += hull.Indices.Length;
            }
        }
        if (totalIndices < 3)
            return null; // hulls from HullFromPoints carry no triangle data

        var positions = new Vector3[totalVertices];
        var indices = new int[totalIndices];
        int vertexOffset = 0, indexOffset = 0;
        foreach (var mesh in hulls.Meshes)
        {
            foreach (var hull in mesh.Hulls)
            {
                var points = hull.Points;
                var hullIndices = hull.Indices;
                for (var i = 0; i < points.Length; i++)
                    positions[vertexOffset + i] = points[i];
                for (var i = 0; i < hullIndices.Length; i++)
                    indices[indexOffset + i] = (int)hullIndices[i] + vertexOffset;
                vertexOffset += points.Length;
                indexOffset += hullIndices.Length;
            }
        }
        return BuildModel(device, positions, indices);
    }

    private SModel BuildModel(GraphicsDevice device, Vector3[] positions, int[] indices)
    {
        var vertices = new VertexPositionNormalTexture[positions.Length];
        for (var i = 0; i < positions.Length; i++)
            vertices[i].Position = positions[i];
        // Accumulated face normals so the default lighting gives the shapes some depth.
        for (var i = 0; i + 2 < indices.Length; i += 3)
        {
            var normal = Vector3.Cross(
                positions[indices[i + 1]] - positions[indices[i]],
                positions[indices[i + 2]] - positions[indices[i]]);
            vertices[indices[i]].Normal += normal;
            vertices[indices[i + 1]].Normal += normal;
            vertices[indices[i + 2]].Normal += normal;
        }
        for (var i = 0; i < vertices.Length; i++)
        {
            if (vertices[i].Normal != Vector3.Zero)
                vertices[i].Normal = Vector3.Normalize(vertices[i].Normal);
        }

        var vertexBuffer = Buffer.Vertex.New(device, vertices);
        var indexBuffer = Buffer.Index.New(device, indices);
        _resources.Add(vertexBuffer);
        _resources.Add(indexBuffer);
        var meshDraw = new MeshDraw
        {
            PrimitiveType = PrimitiveType.TriangleList,
            DrawCount = indices.Length,
            IndexBuffer = new IndexBufferBinding(indexBuffer, is32Bit: true, indices.Length),
            VertexBuffers = new[] { new VertexBufferBinding(vertexBuffer, VertexPositionNormalTexture.Layout, vertices.Length) },
        };
        return new SModel { new Mesh { Draw = meshDraw } };
    }

    private Material GetDefaultMaterial(GraphicsDevice device)
    {
        return _defaultMaterial ??= Material.New(device, new MaterialDescriptor
        {
            Attributes =
            {
                Diffuse = new MaterialDiffuseMapFeature(new ComputeColor(new Color4(0.3f, 0.8f, 1f, 1f))),
                DiffuseModel = new MaterialDiffuseLambertModelFeature(),
                Transparency = new MaterialTransparencyBlendFeature { Alpha = new ComputeFloat(0.25f) },
                CullMode = CullMode.Back,
            },
        });
    }

    public void Dispose()
    {
        foreach (var resource in _resources)
            resource.Dispose();
        _gameHandle.Dispose();
    }
}
