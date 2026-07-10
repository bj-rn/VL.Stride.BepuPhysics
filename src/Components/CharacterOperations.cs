using Stride.Core.Mathematics;
using VL.Core.Import;
using SBepu = global::Stride.BepuPhysics;

namespace VL.Stride.BepuPhysics;

/// <summary>
/// Operations on a Character component. The Character is also a Body, so the Body operations
/// (impulses, Teleport, GetBodyState, ...) work on it as well.
/// Mutating operations run while Apply is true.
/// </summary>
public static class CharacterOperations
{
    /// <summary>Sets the character's movement velocity from Direction times its Speed while Apply is true.</summary>
    /// <param name="character">The character to move.</param>
    /// <param name="direction">Movement direction in world space. The length scales the speed (2 = twice as fast). The last velocity persists, write a zero direction to stop.</param>
    /// <param name="apply">Applies the movement each frame while true. Hold true while driving the character with an input direction.</param>
    [return: Pin(Name = "Output")]
    public static SBepu.CharacterComponent? Move(SBepu.CharacterComponent? character, Vector3 direction, bool apply = false)
    {
        if (apply && character is not null)
        {
            // Wake first, a sleeping character would not integrate the new velocity.
            if (direction != Vector3.Zero)
                character.Awake = true;
            character.Move(direction);
        }
        return character;
    }

    /// <summary>Queues a jump for the next physics tick while Apply is true. Fails silently when the character is not grounded.</summary>
    /// <param name="character">The character that should jump.</param>
    /// <param name="apply">Tries to jump each frame while true. Connect a Bang for a single jump.</param>
    [return: Pin(Name = "Output")]
    public static SBepu.CharacterComponent? TryJump(SBepu.CharacterComponent? character, bool apply = false)
    {
        if (apply && character is not null)
        {
            character.Awake = true;
            character.TryJump();
        }
        return character;
    }

    /// <summary>Reads the character specific settings, mirroring the Character node's inputs. Use GetCollidableInfo for identity and GetBodySettings for the body settings.</summary>
    /// <param name="character">The character to read. Outputs the defaults while null.</param>
    /// <param name="speed">Base movement speed in units per second, scales the direction given to Move.</param>
    /// <param name="jumpForce">Force of the impulse applied by TryJump.</param>
    public static void GetCharacterSettings(SBepu.CharacterComponent? character,
        out float speed, out float jumpForce)
    {
        speed = character?.Speed ?? 10f;
        jumpForce = character?.JumpForce ?? 10f;
    }

    /// <summary>
    /// Writes the character specific settings while Apply is true.
    /// A written property is overwritten again once the owning Character node's pin value changes.
    /// </summary>
    /// <param name="character">The character to write to.</param>
    /// <param name="speed">Base movement speed in units per second, scales the direction given to Move.</param>
    /// <param name="jumpForce">Force of the impulse applied by TryJump.</param>
    /// <param name="apply">Writes all settings each frame while true. Connect a Bang for a one-shot write.</param>
    [return: Pin(Name = "Output")]
    public static SBepu.CharacterComponent? SetCharacterSettings(SBepu.CharacterComponent? character,
        float speed = 10f,
        float jumpForce = 10f,
        bool apply = false)
    {
        if (apply && character is not null)
        {
            character.Speed = speed;
            character.JumpForce = jumpForce;
        }
        return character;
    }

    /// <summary>Reads the character specific state. Use GetBodyState for pose and actual body velocities.</summary>
    /// <param name="character">The character to read. Outputs defaults while not attached to a simulation.</param>
    /// <param name="isGrounded">True while the character stands on a support surface.</param>
    /// <param name="isJumping">True while a jump is in progress.</param>
    /// <param name="movementVelocity">The desired movement velocity set via Move, in world space.</param>
    public static void GetCharacterState(SBepu.CharacterComponent? character,
        out bool isGrounded, out bool isJumping, out Vector3 movementVelocity)
    {
        if (character is null || character.Simulation is null)
        {
            isGrounded = false;
            isJumping = false;
            movementVelocity = Vector3.Zero;
            return;
        }
        isGrounded = character.IsGrounded;
        isJumping = character.IsJumping;
        movementVelocity = character.Velocity;
    }
}
