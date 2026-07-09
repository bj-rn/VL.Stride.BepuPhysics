namespace VL.Stride.BepuPhysics;

/// <summary>
/// Continuous collision detection mode. Mirrors BepuPhysics.Collidables.ContinuousDetectionMode —
/// defined here so the pin default resolves from a VL-imported assembly.
/// </summary>
public enum ContinuousDetectionKind
{
    /// <summary>No sweep tests — fast objects may tunnel through thin geometry.</summary>
    Discrete = 0,
    /// <summary>No sweeps of its own, but other continuous collidables can see it.</summary>
    Passive = 1,
    /// <summary>Sweep tests prevent tunneling at higher cost.</summary>
    Continuous = 2,
}
