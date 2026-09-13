namespace RobotSwarmSimulator;

public sealed class RobotSimulationService
{
    public SimulationMoments Run(SparseHexArena arena, SimulationRequest request, ISimulationRecorder? recorder = null)
    {
        var random = new Random(request.Seed);
        var startState = arena.GetStateId(request.Start);
        var start = arena.DecodeState(startState);
        double sum = 0, squares = 0;
        for (var trial = 1; trial <= request.Trials; trial++)
        {
            var state = startState;
            recorder?.RecordMovement(new(trial, 0, start.X, start.Y, start.Heading, "start"));
            var steps = 0;
            while (state >= 0)
            {
                if (++steps > request.MaxSteps) throw new InvalidOperationException($"Trial {trial} exceeded {request.MaxSteps:N0} steps.");
                state = arena.DrawNextState(state, random);
                if (state < 0)
                    recorder?.RecordMovement(new(trial, steps, arena.Target.X, arena.Target.Y, -1, "target"));
                else
                {
                    var location = arena.DecodeState(state);
                    recorder?.RecordMovement(new(trial, steps, location.X, location.Y, location.Heading, "move"));
                }
            }
            recorder?.RecordTrial(new(trial, steps, true));
            sum += steps;
            squares += (double)steps * steps;
        }
        var mean = sum / request.Trials;
        return new(mean, Math.Max(0, squares / request.Trials - mean * mean), request.Trials);
    }
}
