# Robot Swarm Simulator

This console app is a compact C# migration of the legacy Python robot-swarming work. It covers one robot on an `N x N` arena, an absorbing target, theoretical first-arrival moments, and matching Monte Carlo experiments.

## Run

```powershell
dotnet run --project RobotSwarmSimulator.csproj
```

The app prompts for arena size, target and start coordinates, initial heading, laziness, trial count, and a random seed. Press Enter to accept a default.

By default each run creates `results/experiment_YYYYMMDD_HHMMSS/` with:

- `trial_summary.csv`: one first-arrival time per trial.
- `movements.csv`: every recorded position and heading, including start and target arrival.
- `experiment_summary.csv`: parameters plus theoretical and empirical moments.
- `visit_heatmap.svg`: a portable position-visit heatmap generated from `movements.csv`, similar to the legacy Python heatmap.
- `robot_path_trial_###.mp4`: an optional HD H.264 playback of a selected trial's path.

MP4 animation requires FFmpeg at `tools/ffmpeg/bin/ffmpeg.exe`; this local encoder is intentionally excluded from version control.

The application is separated into `ExperimentController` (console workflow), `SparseHexArena` (domain and theory), `RobotSimulationService` (Monte Carlo execution), `CsvExperimentRecorder` (result persistence), `SvgHeatmapProducer` (heatmap output), and `GifAnimationProducer` (animated playback).

## Sparse implementation

There are `6N²` position-and-heading states, but each state has at most seven outcomes. The C# model stores only reachable successors and their probabilities. It never allocates a dense `(6N²) × (6N²)` array, changing storage from `O(N⁴)` to `O(N²)`.

The solver evaluates `(I-Q)m = 1` and `(I-Q)s = 1 + 2Qm` directly from compact rows; variance is `s - m²`. The simulation samples the same rows, so its estimates converge to theory.

## Location-heading actions

A state is `(x, y, h)`: a cell plus one of six headings. Heading `h` is also the direction used for the move selected on that step.

```text
                 h=2  (-1,+1)      h=1  ( 0,+1)
                           \        /
                            \  cell/
                 h=3  (-1, 0) <---+---> h=0  (+1, 0)
                            /        \
                           /          \
                 h=4  ( 0,-1)      h=5  (+1,-1)
```

Let `L` be the configured laziness probability. Every non-target state first has a **stay at `(x, y, h)`** action with probability `L`. The remaining actions below are multiplied by `1 - L`; their unscaled probabilities sum to 1. A move changes both the cell and heading to the listed direction. If that cell is the target, the transition is absorbing.

`forward`, `backward`, and turns are relative to the current heading. At a boundary, each named preference is assigned to the closest unused legal direction, so no action ever moves through a wall.

| Location-heading state | When it applies | Actions and unscaled probabilities |
| --- | --- | --- |
| I — interior | 6 legal neighboring cells | forward `0.49`; +1 turn `0.24`; +2 turn `0.01`; +3 turn `0.01`; +4 turn `0.01`; -1 turn `0.24` |
| PW — parallel wall | 4 legal neighbors and the current heading is legal | forward `0.45`; backward `0.04`; small turn `0.45`; big turn `0.06` |
| CW — collision wall | 4 legal neighbors and the current heading points into the wall | reflect away from wall `0.60`; backward `0.15`; small turn away from wall `0.15`; remaining legal big turn `0.10` |
| ASC — sharp corner | 2 legal neighbors | backward `0.50`; remaining legal big turn `0.50` |
| AOC — obtuse corner | 3 legal neighbors | backward `0.25`; small turn `0.25`; remaining legal big turn `0.50` |

For example, at an interior state facing heading `0`, the moving probabilities are:

```text
current heading: 0

next heading:       0      1      2      3      4      5
relative action:  forward   +1     +2     +3     +4     -1
probability:      .49    .24    .01    .01    .01    .24

plus: stay in the current state with probability L
each moving probability above is scaled by (1 - L)
```

## Scope

The six headings and the legacy action matrices are retained. Boundary rows assign probability only to legal neighboring cells, so a zero-laziness robot cannot select a wall-directed move; laziness remains the only source of a self-loop. Target arrival absorbs the robot. Multi-robot collisions and dynamic obstacles are intentionally excluded because their dynamics are not represented by single-robot hitting-time theory.
