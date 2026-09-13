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

## Scope

The six headings and the legacy action matrices are retained. Each location-heading state is classified as interior (I), collision with a wall (CW), parallel with a wall (PW), sharp corner (ASC), or obtuse corner (AOC). Boundary rows assign probability only to legal neighboring cells, so a zero-laziness robot cannot select a wall-directed move; laziness remains the only source of a self-loop. Target arrival absorbs the robot. Multi-robot collisions and dynamic obstacles are intentionally excluded because their dynamics are not represented by single-robot hitting-time theory.
