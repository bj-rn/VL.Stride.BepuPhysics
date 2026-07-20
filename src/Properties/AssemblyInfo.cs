using VL.Core.Import;

// Every public type under this namespace (and its sub-namespaces) becomes a node.
// Sub-namespaces extend the category: VL.Stride.BepuPhysics.Colliders -> Stride.Physics.Bepu.Colliders
[assembly: ImportAsIs(Namespace = "VL.Stride.BepuPhysics", Category = "Stride.Physics.Bepu")]

// NOTE: static operation classes always get their CLASS NAME appended as a subcategory
// (an ImportType Category override does NOT suppress this). The class names are therefore
// chosen as user facing category names (Bodies, Queries, Operations), and paired process
// nodes join them via [ProcessNode(Category = ...)].

// Also NOTE: ImportType Category overrides are ignored entirely while ImportAsIs covers
// the assembly (verified for static classes and enums). To place a type into a class based
// category (Bodies, Queries), nest it inside that class.
