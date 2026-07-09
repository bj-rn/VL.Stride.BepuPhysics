using NUnit.Framework;
using VL.TestFramework;

namespace VL.Stride.BepuPhysics.Tests;

/// <summary>
/// Headless compile checks via VL.TestFramework.
/// Currently marked Explicit: VL.Lang's TestEnvironment crashes with
/// ArgumentNullException in ImportedParameterPinDefinitionSymbol.GetDefaultValue when a
/// referenced assembly (e.g. Stride.BepuPhysics itself) has enum parameter defaults —
/// it cannot resolve the enum's runtime type in the headless host. The same documents
/// compile and run fine in real vvvv (use tools/verify-patches.ps1 instead).
/// Candidate for an upstream report to vvvv.
/// </summary>
[TestFixture]
[Explicit("VL.TestFramework cannot resolve enum runtime types of referenced packages — verified working in real vvvv; run tools/verify-patches.ps1 instead.")]
public class PackageTests
{
    private TestEnvironment? _env;

    private static string RepoRoot => Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "..", ".."));
    private static string PackageDir => RepoRoot;

    // The vvvv installation this library targets; override with the VVVV_DIR environment variable.
    private static string VvvvDir =>
        Environment.GetEnvironmentVariable("VVVV_DIR") ?? @"D:\vvvv\vvvv_gamma_7.3-win-x64";

    [OneTimeSetUp]
    public void Setup()
    {
        // Per VL.TestFramework docs: entryAssembly is the path to vvvv.exe —
        // vvvv's own standard libraries are then included automatically.
        // Search paths act like vvvv's --package-repositories directories.
        var searchPaths = new List<string>
        {
            Path.GetDirectoryName(RepoRoot)!, // D:\_dev\_vl-libs (contains this package)
        };
        // Separately installed runtime packages (Stride.BepuPhysics, BepuPhysics, BepuUtilities)
        var userNugets = Environment.ExpandEnvironmentVariables(@"%LOCALAPPDATA%\vvvv\gamma\nugets");
        if (Directory.Exists(userNugets))
            searchPaths.Add(userNugets);

        _env = TestEnvironmentLoader.Load(Path.Combine(VvvvDir, "vvvv.exe"), searchPaths);
    }

    [OneTimeTearDown]
    public void TearDown()
    {
        _env?.Dispose();
        _env = null;
    }

    [Test]
    public async Task MainDocumentCompiles()
    {
        await _env!.LoadAndTestAsync(Path.Combine(PackageDir, "VL.Stride.BepuPhysics.vl"));
    }

    [TestCaseSource(nameof(HelpPatches))]
    public async Task HelpPatchCompiles(string path)
    {
        await _env!.LoadAndTestAsync(path);
    }

    public static IEnumerable<string> HelpPatches()
    {
        var helpDir = Path.Combine(PackageDir, "help");
        if (!Directory.Exists(helpDir))
            yield break;
        foreach (var file in Directory.EnumerateFiles(helpDir, "*.vl", SearchOption.AllDirectories))
            yield return file;
    }
}
