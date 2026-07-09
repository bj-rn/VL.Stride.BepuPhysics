using VL.Core.Import;
using SBepu = global::Stride.BepuPhysics;
using SConstraints = global::Stride.BepuPhysics.Constraints;

namespace VL.Stride.BepuPhysics.Constraints;

/// <summary>Keeps the area of the triangle spanned by three body centers at a target (cloth-like).</summary>
[ProcessNode(Name = "Area")]
public class AreaNode
{
    private readonly SConstraints.AreaConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="bodyC">Third constrained body (C).</param>
    /// <param name="targetScaledArea">Target area of the triangle spanned by the three body centers.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.AreaConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA = null,
        SBepu.BodyComponent? bodyB = null,
        SBepu.BodyComponent? bodyC = null,
        float targetScaledArea = 1f,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true)
    {
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (!ReferenceEquals(_c.C, bodyC)) _c.C = bodyC;
        if (_c.TargetScaledArea != targetScaledArea) _c.TargetScaledArea = targetScaledArea;
        if (_c.SpringFrequency != springFrequency) _c.SpringFrequency = springFrequency;
        if (_c.SpringDampingRatio != springDampingRatio) _c.SpringDampingRatio = springDampingRatio;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Keeps the volume of the tetrahedron spanned by four body centers at a target (softbody-like).</summary>
[ProcessNode(Name = "Volume")]
public class VolumeNode
{
    private readonly SConstraints.VolumeConstraintComponent _c = new();

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="bodyC">Third constrained body (C).</param>
    /// <param name="bodyD">Fourth constrained body (D).</param>
    /// <param name="targetScaledVolume">Target volume of the tetrahedron spanned by the four body centers.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.VolumeConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA = null,
        SBepu.BodyComponent? bodyB = null,
        SBepu.BodyComponent? bodyC = null,
        SBepu.BodyComponent? bodyD = null,
        float targetScaledVolume = 1f,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true)
    {
        if (!ReferenceEquals(_c.A, bodyA)) _c.A = bodyA;
        if (!ReferenceEquals(_c.B, bodyB)) _c.B = bodyB;
        if (!ReferenceEquals(_c.C, bodyC)) _c.C = bodyC;
        if (!ReferenceEquals(_c.D, bodyD)) _c.D = bodyD;
        if (_c.TargetScaledVolume != targetScaledVolume) _c.TargetScaledVolume = targetScaledVolume;
        if (_c.SpringFrequency != springFrequency) _c.SpringFrequency = springFrequency;
        if (_c.SpringDampingRatio != springDampingRatio) _c.SpringDampingRatio = springDampingRatio;
        if (_c.Enabled != enabled) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}
