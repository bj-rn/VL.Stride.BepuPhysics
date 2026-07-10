using System.ComponentModel;
using Stride.Core.Mathematics;
using VL.Core.Import;
using SBepu = global::Stride.BepuPhysics;

namespace VL.Stride.BepuPhysics;

/// <summary>
/// Operations on a Body component. Mutating operations run while Apply is true —
/// connect a Bang for one-shot application.
/// </summary>
public static class BodyOperations
{
    /// <summary>Applies a one-frame impulse at the center of mass while Apply is true.</summary>
    /// <param name="body">The body to apply the impulse to.</param>
    /// <param name="impulse">Impulse in world space (mass times velocity change).</param>
    /// <param name="apply">Applies the impulse each frame while true. Connect a Bang for a one-shot push.</param>
    [return: Pin(Name = "Output")]
    public static SBepu.BodyComponent? ApplyLinearImpulse(SBepu.BodyComponent? body, Vector3 impulse, bool apply = false)
    {
        if (apply && body is not null)
        {
            // Wake first — Stride's impulse and velocity calls write to the BodyReference without
            // waking, and writes to a sleeping body are not integrated.
            body.Awake = true;
            body.ApplyLinearImpulse(impulse);
        }
        return body;
    }

    /// <summary>Applies a one-frame angular impulse while Apply is true.</summary>
    /// <param name="body">The body to apply the impulse to.</param>
    /// <param name="impulse">Angular impulse in world space (spin around each axis).</param>
    /// <param name="apply">Applies the impulse each frame while true. Connect a Bang for a one-shot spin.</param>
    [return: Pin(Name = "Output")]
    public static SBepu.BodyComponent? ApplyAngularImpulse(SBepu.BodyComponent? body, Vector3 impulse, bool apply = false)
    {
        if (apply && body is not null)
        {
            body.Awake = true;
            body.ApplyAngularImpulse(impulse);
        }
        return body;
    }

    /// <summary>Applies an impulse at an offset from the center of mass while Apply is true.</summary>
    /// <param name="body">The body to apply the impulse to.</param>
    /// <param name="impulse">Impulse in world space (mass times velocity change).</param>
    /// <param name="impulseOffset">Application point relative to the center of mass — off-center impulses add spin.</param>
    /// <param name="apply">Applies the impulse each frame while true. Connect a Bang for a one-shot push.</param>
    [return: Pin(Name = "Output")]
    public static SBepu.BodyComponent? ApplyImpulse(SBepu.BodyComponent? body, Vector3 impulse, Vector3 impulseOffset, bool apply = false)
    {
        if (apply && body is not null)
        {
            body.Awake = true;
            body.ApplyImpulse(impulse, impulseOffset);
        }
        return body;
    }

    /// <summary>Instantly moves the body while Apply is true (no sweep, no collision on the way).</summary>
    /// <param name="body">The body to move.</param>
    /// <param name="position">Target position in world space.</param>
    /// <param name="orientation">Target orientation in world space. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="apply">Teleports each frame while true. Connect a Bang. Collisions along the way are ignored.</param>
    [return: Pin(Name = "Output")]
    public static SBepu.BodyComponent? Teleport(SBepu.BodyComponent? body, Vector3 position,
        Quaternion orientation, bool apply = false)
    {
        if (apply && body is not null)
        {
            body.Awake = true;
            body.Teleport(position, orientation);
        }
        return body;
    }

    /// <summary>Sets the target pose for a kinematic body while Apply is true — the body moves there with proper sweep.</summary>
    /// <param name="body">The kinematic body to move.</param>
    /// <param name="position">Target position in world space.</param>
    /// <param name="orientation">Target orientation in world space. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="apply">Moves towards the target with proper sweep each frame while true — pushes dynamic bodies out of the way.</param>
    [return: Pin(Name = "Output")]
    public static SBepu.BodyComponent? SetTargetPose(SBepu.BodyComponent? body, Vector3 position,
        Quaternion orientation, bool apply = false)
    {
        if (apply && body is not null)
        {
            body.Awake = true;
            body.SetTargetPose(position, orientation);
        }
        return body;
    }

    /// <summary>Overwrites the linear velocity while Apply is true.</summary>
    /// <param name="body">The body whose velocity is overwritten.</param>
    /// <param name="velocity">New linear velocity in units per second, world space.</param>
    /// <param name="apply">Overwrites each frame while true. Connect a Bang for a one-shot set.</param>
    [return: Pin(Name = "Output")]
    public static SBepu.BodyComponent? SetLinearVelocity(SBepu.BodyComponent? body, Vector3 velocity, bool apply = false)
    {
        if (apply && body is not null)
        {
            body.Awake = true;
            body.LinearVelocity = velocity;
        }
        return body;
    }

    /// <summary>Overwrites the angular velocity while Apply is true.</summary>
    /// <param name="body">The body whose velocity is overwritten.</param>
    /// <param name="velocity">New angular velocity in radians per second around each world axis.</param>
    /// <param name="apply">Overwrites each frame while true. Connect a Bang for a one-shot set.</param>
    [return: Pin(Name = "Output")]
    public static SBepu.BodyComponent? SetAngularVelocity(SBepu.BodyComponent? body, Vector3 velocity, bool apply = false)
    {
        if (apply && body is not null)
        {
            body.Awake = true;
            body.AngularVelocity = velocity;
        }
        return body;
    }

    /// <summary>Reads the current physics state of a body.</summary>
    /// <param name="body">The body to read. Outputs defaults while not attached to a simulation.</param>
    /// <param name="position">Current position in world space.</param>
    /// <param name="orientation">Current orientation in world space. Use identity (0, 0, 0, 1) for no rotation, an all zero quaternion is invalid.</param>
    /// <param name="linearVelocity">Current linear velocity in units per second.</param>
    /// <param name="angularVelocity">Current angular velocity in radians per second.</param>
    /// <param name="awake">True while the body is actively simulated (not sleeping).</param>
    public static void GetBodyState(SBepu.BodyComponent? body,
        out Vector3 position, out Quaternion orientation,
        out Vector3 linearVelocity, out Vector3 angularVelocity, out bool awake)
    {
        if (body is null || body.Simulation is null)
        {
            position = Vector3.Zero;
            orientation = Quaternion.Identity;
            linearVelocity = Vector3.Zero;
            angularVelocity = Vector3.Zero;
            awake = false;
            return;
        }
        position = body.Position;
        orientation = body.Orientation;
        linearVelocity = body.LinearVelocity;
        angularVelocity = body.AngularVelocity;
        awake = body.Awake;
    }

    /// <summary>Wakes the body up while Apply is true.</summary>
    /// <param name="body">The body to wake up.</param>
    /// <param name="apply">Wakes the body each frame while true.</param>
    [return: Pin(Name = "Output")]
    public static SBepu.BodyComponent? Awaken(SBepu.BodyComponent? body, bool apply = false)
    {
        if (apply && body is not null)
            body.Awake = true;
        return body;
    }
}
