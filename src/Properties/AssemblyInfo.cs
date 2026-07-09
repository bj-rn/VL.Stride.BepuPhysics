using VL.Core.Import;

// Every public type under this namespace (and its sub-namespaces) becomes a node.
// Sub-namespaces extend the category: VL.Stride.BepuPhysics.Colliders -> Stride.Physics.Bepu.Colliders
[assembly: ImportAsIs(Namespace = "VL.Stride.BepuPhysics", Category = "Stride.Physics.Bepu")]
