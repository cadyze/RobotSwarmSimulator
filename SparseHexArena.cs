namespace RobotSwarmSimulator;

/// <summary>A memory-efficient, single-robot version of the legacy six-heading arena.</summary>
public sealed class SparseHexArena
{
    private static readonly (int X, int Y)[] Steps = [(1, 0), (0, 1), (-1, 1), (-1, 0), (0, -1), (1, -1)];
    // These are the I, CW, PW, ASC, and AOC action matrices from the legacy
    // Python simulator.  Boundary actions are assigned only to legal neighbors.
    private static readonly double[] InteriorActions = [0.49, 0.24, 0.01, 0.01, 0.01, 0.24];
    private const double CollisionReflect = 0.60;
    private const double CollisionBackward = 0.15;
    private const double CollisionSmallTurn = 0.15;
    private const double CollisionBigTurn = 0.10;
    private const double ParallelForward = 0.45;
    private const double ParallelBackward = 0.04;
    private const double ParallelSmallTurn = 0.45;
    private const double ParallelBigTurn = 0.06;
    private const double SharpCornerBackward = 0.50;
    private const double SharpCornerBigTurn = 0.50;
    private const double ObtuseCornerBackward = 0.25;
    private const double ObtuseCornerSmallTurn = 0.25;
    private const double ObtuseCornerBigTurn = 0.50;
    private readonly Transition[][] _transitions;
    private readonly bool[] _targetStates;

    public int Size { get; }
    public (int X, int Y) Target { get; }
    public double Laziness { get; }
    public int StateCount => 6 * Size * Size;
    public int StoredTransitionCount { get; }

    public SparseHexArena(int size, (int X, int Y) target, double laziness = 0)
    {
        if (size < 3) throw new ArgumentOutOfRangeException(nameof(size), "Arena size must be at least 3.");
        if (laziness is < 0 or >= 1) throw new ArgumentOutOfRangeException(nameof(laziness), "Laziness must be in [0, 1).");
        if (!Inside(target.X, target.Y, size)) throw new ArgumentOutOfRangeException(nameof(target));
        Size = size; Target = target; Laziness = laziness;
        _transitions = new Transition[StateCount][];
        _targetStates = new bool[StateCount];
        BuildTransitions();
        ValidateTransitions();
        StoredTransitionCount = _transitions.Sum(row => row.Length);
    }

    public HittingTimeMoments CalculateHittingTimeMoments((int X, int Y, int Heading) start, double tolerance = 1e-10, int maxIterations = 200_000)
    {
        var startState = StateId(start.X, start.Y, start.Heading);
        if (_targetStates[startState]) return new(0, 0, 0, 0);
        var mean = new double[StateCount];
        var meanRhs = BuildActiveRhs();
        var meanIterations = Solve(mean, meanRhs, tolerance, maxIterations);
        var second = new double[StateCount];
        var secondRhs = BuildSecondMomentRhs(mean);
        var secondIterations = Solve(second, secondRhs, tolerance, maxIterations);
        return new(mean[startState], Math.Max(0, second[startState] - mean[startState] * mean[startState]), meanIterations, secondIterations);
    }

    public SimulationMoments Simulate((int X, int Y, int Heading) start, int trials, int seed, int maxSteps = 1_000_000)
    {
        if (trials < 1) throw new ArgumentOutOfRangeException(nameof(trials));
        var startState = StateId(start.X, start.Y, start.Heading);
        if (_targetStates[startState]) return new(0, 0, trials);
        var random = new Random(seed);
        double sum = 0, squares = 0;
        for (var trial = 0; trial < trials; trial++)
        {
            var state = startState; var steps = 0;
            do
            {
                if (++steps > maxSteps) throw new InvalidOperationException($"Trial {trial + 1} exceeded {maxSteps:N0} steps.");
                state = DrawNextState(state, random);
            } while (state >= 0);
            sum += steps; squares += (double)steps * steps;
        }
        var mean = sum / trials;
        return new(mean, Math.Max(0, squares / trials - mean * mean), trials);
    }

    private double[] BuildActiveRhs()
    {
        var rhs = new double[StateCount];
        for (var state = 0; state < StateCount; state++) if (!_targetStates[state]) rhs[state] = 1;
        return rhs;
    }

    private double[] BuildSecondMomentRhs(double[] mean)
    {
        var rhs = BuildActiveRhs();
        for (var state = 0; state < StateCount; state++)
        {
            if (_targetStates[state]) continue;
            foreach (var transition in _transitions[state])
                if (transition.NextState >= 0) rhs[state] += 2 * transition.Probability * mean[transition.NextState];
        }
        return rhs;
    }

    private int Solve(double[] values, double[] rhs, double tolerance, int maxIterations)
    {
        for (var iteration = 1; iteration <= maxIterations; iteration++)
        {
            var maximumChange = 0.0;
            for (var state = 0; state < StateCount; state++)
            {
                if (_targetStates[state]) continue;
                var self = 0.0; var others = 0.0;
                foreach (var transition in _transitions[state])
                    if (transition.NextState < 0) continue; // Absorbing target has value zero.
                    else if (transition.NextState == state) self += transition.Probability;
                    else others += transition.Probability * values[transition.NextState];
                var next = (rhs[state] + others) / (1 - self);
                maximumChange = Math.Max(maximumChange, Math.Abs(next - values[state]));
                values[state] = next;
            }
            if (maximumChange <= tolerance) return iteration;
        }
        throw new InvalidOperationException($"Sparse solver exceeded {maxIterations:N0} iterations.");
    }

    internal int DrawNextState(int state, Random random)
    {
        var draw = random.NextDouble(); var cumulative = 0.0;
        foreach (var transition in _transitions[state])
        {
            cumulative += transition.Probability;
            if (draw < cumulative) return transition.NextState;
        }
        return _transitions[state][^1].NextState;
    }

    private void BuildTransitions()
    {
        var scale = 1 - Laziness;
        for (var y = 0; y < Size; y++) for (var x = 0; x < Size; x++) for (var heading = 0; heading < 6; heading++)
        {
            var state = StateId(x, y, heading);
            if ((x, y) == Target) { _targetStates[state] = true; _transitions[state] = []; continue; }
            var outcomes = new Dictionary<int, double>();
            Add(outcomes, state, Laziness);
            var legalDirections = LegalDirections(x, y);
            foreach (var (nextHeading, probability) in BoundaryActions(heading, legalDirections))
            {
                var (dx, dy) = Steps[nextHeading];
                var nextX = x + dx; var nextY = y + dy;
                if ((nextX, nextY) == Target) Add(outcomes, -1, scale * probability);
                else Add(outcomes, StateId(nextX, nextY, nextHeading), scale * probability);
            }
            if (Math.Abs(outcomes.Values.Sum() - 1) > 1e-12) throw new InvalidOperationException("Transition row does not sum to one.");
            _transitions[state] = outcomes.Select(pair => new Transition(pair.Key, pair.Value)).ToArray();
        }
    }

    private List<int> LegalDirections(int x, int y) =>
        Enumerable.Range(0, 6).Where(direction =>
        {
            var (dx, dy) = Steps[direction];
            return Inside(x + dx, y + dy, Size);
        }).ToList();

    private IEnumerable<(int Direction, double Probability)> BoundaryActions(int heading, List<int> legal)
    {
        // The Python source has TODO/uninitialized rows for some boundary headings.
        // These rules preserve its five action matrices while completing those rows by
        // rotational symmetry, so every state has a normalized, legal transition row.
        return legal.Count switch
        {
            6 => InteriorActions.Select((probability, turn) => ((heading + turn) % 6, probability)),
            4 when legal.Contains(heading) => ParallelWallActions(heading, legal),
            4 => CollisionWallActions(heading, legal),
            3 => ObtuseCornerActions(heading, legal),
            2 => SharpCornerActions(heading, legal),
            _ => throw new InvalidOperationException("Arena cell has an unsupported boundary shape.")
        };
    }

    private static IEnumerable<(int Direction, double Probability)> ParallelWallActions(int heading, List<int> legal)
    {
        var available = new List<int>(legal);
        yield return (TakeClosest(available, heading), ParallelForward);
        yield return (TakeClosest(available, (heading + 3) % 6), ParallelBackward);
        yield return (TakeClosest(available, (heading + 1) % 6), ParallelSmallTurn);
        yield return (available.Single(), ParallelBigTurn);
    }

    private static IEnumerable<(int Direction, double Probability)> CollisionWallActions(int heading, List<int> legal)
    {
        var invalid = Enumerable.Range(0, 6).Where(direction => !legal.Contains(direction)).ToList();
        var firstInvalid = invalid.First(direction => invalid.Contains((direction + 1) % 6));
        var away = heading == firstInvalid ? -1 : 1;
        var available = new List<int>(legal);
        yield return (TakeClosest(available, Mod(heading - 4 * away)), CollisionReflect);
        yield return (TakeClosest(available, Mod(heading - 3 * away)), CollisionBackward);
        yield return (TakeClosest(available, Mod(heading + away)), CollisionSmallTurn);
        yield return (available.Single(), CollisionBigTurn);
    }

    private static IEnumerable<(int Direction, double Probability)> SharpCornerActions(int heading, List<int> legal)
    {
        var available = new List<int>(legal);
        yield return (TakeClosest(available, (heading + 3) % 6), SharpCornerBackward);
        yield return (available.Single(), SharpCornerBigTurn);
    }

    private static IEnumerable<(int Direction, double Probability)> ObtuseCornerActions(int heading, List<int> legal)
    {
        var available = new List<int>(legal);
        yield return (TakeClosest(available, (heading + 3) % 6), ObtuseCornerBackward);
        yield return (TakeClosest(available, (heading + 1) % 6), ObtuseCornerSmallTurn);
        yield return (available.Single(), ObtuseCornerBigTurn);
    }

    private static int TakeClosest(List<int> available, int preferred)
    {
        var selected = available.OrderBy(direction => CircularDistance(direction, preferred)).ThenBy(direction => direction).First();
        available.Remove(selected);
        return selected;
    }

    private static int CircularDistance(int left, int right) => Math.Min(Mod(left - right), Mod(right - left));
    private static int Mod(int value) => (value % 6 + 6) % 6;

    private void ValidateTransitions()
    {
        for (var state = 0; state < StateCount; state++)
        {
            if (_targetStates[state]) continue;
            var source = DecodeState(state);
            foreach (var transition in _transitions[state])
            {
                if (transition.NextState < 0) continue;
                if (transition.NextState == state && Laziness > 0) continue;
                var destination = DecodeState(transition.NextState);
                var (dx, dy) = Steps[destination.Heading];
                if (destination.X != source.X + dx || destination.Y != source.Y + dy)
                    throw new InvalidOperationException("Transition matrix contains a non-neighbor or wall-directed move.");
            }
        }
    }

    private static void Add(Dictionary<int, double> outcomes, int state, double probability)
    {
        if (probability != 0) outcomes[state] = outcomes.GetValueOrDefault(state) + probability;
    }
    private int StateId(int x, int y, int heading)
    {
        if (!Inside(x, y, Size) || heading is < 0 or > 5) throw new ArgumentOutOfRangeException(nameof(heading));
        return 6 * (x + Size * y) + heading;
    }
    internal (int X, int Y, int Heading) DecodeState(int state)
    {
        var heading = state % 6;
        var cell = state / 6;
        return (cell % Size, cell / Size, heading);
    }
    internal int GetStateId((int X, int Y, int Heading) state) => StateId(state.X, state.Y, state.Heading);
    private static bool Inside(int x, int y, int size) => x >= 0 && x < size && y >= 0 && y < size;
    private readonly record struct Transition(int NextState, double Probability);
}

public readonly record struct HittingTimeMoments(double Mean, double Variance, int MeanIterations, int SecondMomentIterations);
public readonly record struct SimulationMoments(double Mean, double Variance, int Trials);
