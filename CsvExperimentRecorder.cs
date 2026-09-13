using System.Globalization;
using System.IO;

namespace RobotSwarmSimulator;

public sealed class CsvExperimentRecorder : ISimulationRecorder
{
    private readonly StreamWriter _movementWriter;
    private readonly StreamWriter _trialWriter;
    public string OutputDirectory { get; }
    public string MovementsCsvPath { get; }
    public string TrialSummaryCsvPath { get; }

    public CsvExperimentRecorder(string outputDirectory)
    {
        OutputDirectory = outputDirectory;
        Directory.CreateDirectory(outputDirectory);
        MovementsCsvPath = Path.Combine(outputDirectory, "movements.csv");
        TrialSummaryCsvPath = Path.Combine(outputDirectory, "trial_summary.csv");
        _movementWriter = new StreamWriter(MovementsCsvPath);
        _trialWriter = new StreamWriter(TrialSummaryCsvPath);
        _movementWriter.WriteLine("trial,step,x,y,heading,event");
        _trialWriter.WriteLine("trial,steps,reached_target");
    }

    public void RecordMovement(RobotMovement movement) => _movementWriter.WriteLine(string.Join(',', movement.Trial, movement.Step, movement.X, movement.Y, movement.Heading, movement.Event));
    public void RecordTrial(TrialResult result) => _trialWriter.WriteLine(string.Join(',', result.Trial, result.Steps, result.ReachedTarget.ToString().ToLowerInvariant()));

    public void WriteMetadata(SparseHexArena arena, SimulationRequest request, HittingTimeMoments theory, SimulationMoments simulation)
    {
        File.WriteAllLines(Path.Combine(OutputDirectory, "experiment_summary.csv"),
        [
            "field,value",
            $"arena_size,{arena.Size}", $"target_x,{arena.Target.X}", $"target_y,{arena.Target.Y}",
            $"start_x,{request.Start.X}", $"start_y,{request.Start.Y}", $"start_heading,{request.Start.Heading}",
            $"laziness,{arena.Laziness.ToString(CultureInfo.InvariantCulture)}", $"trials,{request.Trials}", $"seed,{request.Seed}",
            $"theory_mean,{theory.Mean.ToString(CultureInfo.InvariantCulture)}", $"theory_variance,{theory.Variance.ToString(CultureInfo.InvariantCulture)}",
            $"simulation_mean,{simulation.Mean.ToString(CultureInfo.InvariantCulture)}", $"simulation_variance,{simulation.Variance.ToString(CultureInfo.InvariantCulture)}"
        ]);
    }
    public void Flush() { _movementWriter.Flush(); _trialWriter.Flush(); }
    public void Dispose() { _movementWriter.Dispose(); _trialWriter.Dispose(); }
}
