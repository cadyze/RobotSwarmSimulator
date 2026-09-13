namespace RobotSwarmSimulator;

public readonly record struct SimulationRequest((int X, int Y, int Heading) Start, int Trials, int Seed, int MaxSteps = 1_000_000);
public readonly record struct RobotMovement(int Trial, int Step, int X, int Y, int Heading, string Event);
public readonly record struct TrialResult(int Trial, int Steps, bool ReachedTarget);
public interface ISimulationRecorder : IDisposable
{
    void RecordMovement(RobotMovement movement);
    void RecordTrial(TrialResult result);
}
