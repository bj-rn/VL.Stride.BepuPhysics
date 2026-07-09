using System.Runtime.CompilerServices;
using Stride.Data;
using Stride.Engine;
using Stride.Engine.Design;
using Stride.Games;
using VL.Core;
using SBepu = global::Stride.BepuPhysics;

namespace VL.Stride.BepuPhysics.Internal;

/// <summary>
/// Quietly pre-configures Bepu for the running game: without a BepuConfiguration entry in the
/// game settings, the engine's lazy bootstrap (BepuConfiguration.NewInstance) logs two warnings
/// ("Creating a default configuration..." / "No simulations configured...") before creating
/// defaults anyway. We inject the same defaults into the settings up front, so the engine finds
/// them and stays silent — while still registering its (internal) PhysicsGameSystem itself.
/// </summary>
internal static class BepuSettingsBootstrap
{
    private static readonly ConditionalWeakTable<Game, object> Configured = new();

    public static void EnsureConfigured()
    {
        try
        {
            using var gameHandle = AppHost.Current.Services.GetGameHandle();
            var game = gameHandle.Resource;

            lock (Configured)
            {
                if (Configured.TryGetValue(game, out _))
                    return;
                Configured.Add(game, new object());
            }

            // Already bootstrapped by the engine (or a previous hot-reload generation)?
            if (game.Services.GetService<SBepu.BepuConfiguration>() is not null)
                return;

            var configurations = game.Services.GetService<IGameSettingsService>()?.Settings?.Configurations;
            if (configurations is null)
                return; // no settings service — the engine's fallback path is silent in that case

            foreach (var entry in configurations.Configurations)
            {
                if (entry.Configuration is SBepu.BepuConfiguration)
                    return; // someone already configured Bepu
            }

            configurations.Configurations.Add(new ConfigurationOverride
            {
                Platforms = ConfigPlatforms.None,
                SpecificFilter = -1,
                Configuration = new SBepu.BepuConfiguration
                {
                    BepuSimulations = { new SBepu.BepuSimulation() },
                },
            });
        }
        catch
        {
            // No game in this app host — nothing to configure; the engine handles it if physics
            // is used anyway (with its warnings).
        }
    }
}
