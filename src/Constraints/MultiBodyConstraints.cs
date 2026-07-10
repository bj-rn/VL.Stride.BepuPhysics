using VL.Core.Import;
using VL.Model;
using VL.Stride.BepuPhysics.Internal;
using SBepu = global::Stride.BepuPhysics;
using SConstraints = global::Stride.BepuPhysics.Constraints;

namespace VL.Stride.BepuPhysics.Constraints;

/// <summary>Keeps the area of the triangle spanned by three body centers at a target (cloth-like).</summary>
[ProcessNode(Name = "Area")]
public class AreaNode
{
    private readonly SConstraints.AreaConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<SBepu.BodyComponent?> _bodyC;
    private PinValue<float> _targetScaledArea;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="bodyC">Third constrained body (C).</param>
    /// <param name="targetScaledArea">Target area of the triangle spanned by the three body centers.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
    [return: Pin(Name = "Output")]
    public SConstraints.AreaConstraintComponent Update(
        out bool attached,
        SBepu.BodyComponent? bodyA = null,
        SBepu.BodyComponent? bodyB = null,
        SBepu.BodyComponent? bodyC = null,
        float targetScaledArea = 1f,
        float springFrequency = 30f,
        float springDampingRatio = 5f,
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_bodyC.Changed(bodyC) | reapplyInputs) _c.C = bodyC;
        if (_targetScaledArea.Changed(targetScaledArea) | reapplyInputs) _c.TargetScaledArea = targetScaledArea;
        if (_springFrequency.Changed(springFrequency) | reapplyInputs) _c.SpringFrequency = springFrequency;
        if (_springDampingRatio.Changed(springDampingRatio) | reapplyInputs) _c.SpringDampingRatio = springDampingRatio;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}

/// <summary>Keeps the volume of the tetrahedron spanned by four body centers at a target (softbody-like).</summary>
[ProcessNode(Name = "Volume")]
public class VolumeNode
{
    private readonly SConstraints.VolumeConstraintComponent _c = new();
    // Change detection is against the last PIN value, not the component state, so setter
    // nodes may mutate the component without this node reverting it (see PinValue<T>).
    private PinValue<SBepu.BodyComponent?> _bodyA;
    private PinValue<SBepu.BodyComponent?> _bodyB;
    private PinValue<SBepu.BodyComponent?> _bodyC;
    private PinValue<SBepu.BodyComponent?> _bodyD;
    private PinValue<float> _targetScaledVolume;
    private PinValue<float> _springFrequency;
    private PinValue<float> _springDampingRatio;
    private PinValue<bool> _enabled;

    /// <param name="attached">True while the constraint is active in the simulation (bodies valid, same simulation, enabled).</param>
    /// <param name="bodyA">First constrained body (A).</param>
    /// <param name="bodyB">Second constrained body (B).</param>
    /// <param name="bodyC">Third constrained body (C).</param>
    /// <param name="bodyD">Fourth constrained body (D).</param>
    /// <param name="targetScaledVolume">Target volume of the tetrahedron spanned by the four body centers.</param>
    /// <param name="springFrequency">Constraint spring stiffness in Hz (target undamped oscillation frequency).</param>
    /// <param name="springDampingRatio">Constraint spring damping; 1 = critical damping, higher settles stiffer.</param>
    /// <param name="enabled">Temporarily deactivates the constraint when false.</param>
    /// <param name="reapplyInputs">While true, writes all input values to the component again, overriding values written by setter nodes. Connect a Bang.</param>
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
        bool enabled = true,
        [Pin(Visibility = PinVisibility.Optional)] bool reapplyInputs = false)
    {
        // Non-short-circuit | so the shadow fields update even while Reapply Inputs is true.
        if (_bodyA.Changed(bodyA) | reapplyInputs) _c.A = bodyA;
        if (_bodyB.Changed(bodyB) | reapplyInputs) _c.B = bodyB;
        if (_bodyC.Changed(bodyC) | reapplyInputs) _c.C = bodyC;
        if (_bodyD.Changed(bodyD) | reapplyInputs) _c.D = bodyD;
        if (_targetScaledVolume.Changed(targetScaledVolume) | reapplyInputs) _c.TargetScaledVolume = targetScaledVolume;
        if (_springFrequency.Changed(springFrequency) | reapplyInputs) _c.SpringFrequency = springFrequency;
        if (_springDampingRatio.Changed(springDampingRatio) | reapplyInputs) _c.SpringDampingRatio = springDampingRatio;
        if (_enabled.Changed(enabled) | reapplyInputs) _c.Enabled = enabled;
        attached = _c.Attached;
        return _c;
    }
}
