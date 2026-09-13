using System.Globalization;
using System.IO;

namespace RobotSwarmSimulator;

/// <summary>Owns the console workflow; numerical and file concerns remain in services.</summary>
public sealed class ExperimentController
{
    private readonly RobotSimulationService _simulation = new();
    private readonly SvgHeatmapProducer _graphs = new();
    private readonly GifAnimationProducer _animations = new();

    public void Run()
    {
        Console.WriteLine("Sparse single-robot hex-arena search simulator");
        Console.WriteLine("The model compares absorbing Markov-chain theory with Monte Carlo trials.\n");
        do RunExperiment(); while (AskYesNo("\nRun another experiment?", false));
    }

    private void RunExperiment()
    {
        var size = AskInt("Arena size N", 31, 3);
        var targetX = AskInt("Target x", size / 2, 0, size - 1);
        var targetY = AskInt("Target y", size / 2, 0, size - 1);
        var startX = AskInt("Start x", 0, 0, size - 1);
        var startY = AskInt("Start y", 0, 0, size - 1);
        var heading = AskInt("Start heading (0-5)", 0, 0, 5);
        var laziness = AskDouble("Laziness probability", 0, 0, 0.999999);
        var trials = AskInt("Monte Carlo trials", 10_000, 1);
        var seed = AskInt("Random seed", 20260913, int.MinValue, int.MaxValue);
        var saveOutput = AskYesNo("Save detailed CSV results and SVG heatmap", true);

        var arena = new SparseHexArena(size, (targetX, targetY), laziness);
        var request = new SimulationRequest((startX, startY, heading), trials, seed);
        Console.WriteLine($"\nBuilt {arena.StateCount:N0} states and {arena.StoredTransitionCount:N0} stored transitions.");
        try
        {
            var theory = arena.CalculateHittingTimeMoments(request.Start);
            SimulationMoments simulation;
            if (saveOutput)
            {
                var folder = Path.Combine(FindProjectRoot(), "results", $"experiment_{DateTime.Now:yyyyMMdd_HHmmss_fff}");
                string movementCsv;
                using (var recorder = new CsvExperimentRecorder(folder))
                {
                    simulation = _simulation.Run(arena, request, recorder);
                    recorder.WriteMetadata(arena, request, theory, simulation);
                    movementCsv = recorder.MovementsCsvPath;
                }
                var graph = _graphs.Create(movementCsv, size, arena.Target, Path.Combine(folder, "visit_heatmap.svg"));
                if (AskYesNo("Create an MP4 animation from a trial", true))
                {
                    var animationTrial = AskInt("Trial to animate", 1, 1, trials);
                    try
                    {
                        var animation = _animations.Create(movementCsv, size, arena.Target, animationTrial, Path.Combine(folder, $"robot_path_trial_{animationTrial:D3}.mp4"));
                        Console.WriteLine($"Animation: {Path.GetFullPath(animation)}");
                    }
                    catch (Exception error)
                    {
                        Console.WriteLine($"MP4 animation was not created: {error.Message}");
                    }
                }
                Console.WriteLine($"Detailed files saved to {Path.GetFullPath(folder)}");
                Console.WriteLine($"Heatmap: {Path.GetFullPath(graph)}");
            }
            else simulation = _simulation.Run(arena, request);

            Console.WriteLine("\nFirst-arrival time (steps)");
            Console.WriteLine($"  Theoretical mean:       {theory.Mean:F4}");
            Console.WriteLine($"  Theoretical variance:   {theory.Variance:F4}");
            Console.WriteLine($"  Simulation mean:        {simulation.Mean:F4}");
            Console.WriteLine($"  Simulation variance:    {simulation.Variance:F4}");
            Console.WriteLine($"  Solver iterations:      mean {theory.MeanIterations:N0}, second moment {theory.SecondMomentIterations:N0}");
        }
        catch (InvalidOperationException error) { Console.WriteLine($"\nCalculation failed: {error.Message}"); }
    }

    private static int AskInt(string label, int defaultValue, int minimum, int maximum = int.MaxValue)
    {
        while (true)
        {
            Console.Write($"{label} [{defaultValue}]: "); var text = Console.ReadLine();
            if (string.IsNullOrWhiteSpace(text)) return defaultValue;
            if (int.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture, out var value) && value >= minimum && value <= maximum) return value;
            Console.WriteLine($"Enter an integer from {minimum} to {maximum}.");
        }
    }
    private static double AskDouble(string label, double defaultValue, double minimum, double maximum)
    {
        while (true)
        {
            Console.Write($"{label} [{defaultValue.ToString(CultureInfo.InvariantCulture)}]: "); var text = Console.ReadLine();
            if (string.IsNullOrWhiteSpace(text)) return defaultValue;
            if (double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out var value) && value >= minimum && value <= maximum) return value;
            Console.WriteLine($"Enter a number from {minimum} to {maximum}.");
        }
    }
    private static bool AskYesNo(string label, bool defaultValue)
    {
        var suffix = defaultValue ? "[Y/n]" : "[y/N]";
        while (true)
        {
            Console.Write($"{label} {suffix}: "); var text = Console.ReadLine()?.Trim();
            if (string.IsNullOrEmpty(text)) return defaultValue;
            if (text.Equals("y", StringComparison.OrdinalIgnoreCase) || text.Equals("yes", StringComparison.OrdinalIgnoreCase)) return true;
            if (text.Equals("n", StringComparison.OrdinalIgnoreCase) || text.Equals("no", StringComparison.OrdinalIgnoreCase)) return false;
            Console.WriteLine("Enter y or n.");
        }
    }

    private static string FindProjectRoot()
    {
        foreach (var start in new[] { Directory.GetCurrentDirectory(), AppContext.BaseDirectory })
            for (var directory = new DirectoryInfo(start); directory is not null; directory = directory.Parent)
                if (File.Exists(Path.Combine(directory.FullName, "RobotSwarmSimulator.csproj"))) return directory.FullName;
        return Directory.GetCurrentDirectory();
    }
}
