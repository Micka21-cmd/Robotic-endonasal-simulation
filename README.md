# Simulation-Based Motion Planning for a Surgical Robot in a Constrained Environment

Academic project by **Mickaël Ryon — Polytech Dijon**, October 2026.

This project compares RRT and RRT* for post-insertion motion of a rigid instrument attached to a UR3 robot. Planning takes place in a synthetic corridor inspired by an endonasal access, with a fixed Remote Center of Motion (RCM) and additional obstacles. CoppeliaSim runs the simulation and Lua implementation; MATLAB checks the exported data and produces the comparative figures and statistics.

The instrument is already inserted at the start of each run. External approach and insertion through the entry point are outside the scope of this project.

## 1. Requirements

- **CoppeliaSim Edu 4.10.0 revision 0**, used for the reported experiments on Windows.
- The CoppeliaSim **simIK plugin**, available through `require 'simIK'`.
- **MATLAB** to run the analysis scripts. The scripts use functions including `readtable`, `writetable`, `tiledlayout`, and `exportgraphics`; a release supporting these functions is required. The exact MATLAB release should be recorded with `version` when reproducing the analysis.
- A local copy of this repository and write access to its `Results/` and `analysis/outputs/` directories.

The evaluated RRT and RRT* implementations are Lua modules in this repository. The earlier RRT-Connect/OMPL test bench is development history, not a third method in the two reported campaigns.

## 2. Repository layout

| Path | Purpose |
| --- | --- |
| `Scenes/endonasal_V0_posture_B_obstacle_O1.ttt` | Arrangement B with obstacle O1 |
| `Scenes/endonasal_V0_posture_B_obstacle_O2.ttt` | Arrangement B with obstacle O2 |
| `Scenes/Scripts/Script.lua` | Source copy of the UR3 simulation entry script |
| `src/config.lua` | Target, execution mode, planner settings, diagnostics and exports |
| `src/experiment.lua` | Experiment initialization, execution and cleanup |
| `src/robot.lua`, `src/kinematics.lua` | Robot access and inverse kinematics |
| `src/trajectory.lua` | Prescribed reference motion |
| `src/validation.lua`, `src/segment_validation.lua` | State checks and intermediate segment sampling |
| `src/local_connector.lua` | IK-based local connection and fixed-node connection |
| `src/rrt.lua`, `src/rrt_star.lua` | Evaluated planners |
| `src/profiler.lua` | Planning instrumentation |
| `src/run_directory.lua`, `src/export.lua` | Run directories and CSV exports |
| `Results/` | Timestamped experimental runs, including planning failures |
| `analysis/analyze_T2_O1.m` | CMP01 analysis |
| `analysis/analyze_T1_O2.m` | CMP02 analysis |
| `analysis/outputs/` | Analysis tables, notes and PDF/PNG figures |

## 3. Model and validation scope

The model uses a rigid instrument of length **233 mm** and diameter **3.5 mm**. The synthetic V0 corridor has a 2 mm radial wall thickness and 96 angular sectors. Its inner profile is defined by the following axial positions and radii:

| Axial position (mm) | 0 | 27 | 54 | 65 | 90 |
| --- | --- | --- | --- | --- | --- |
| Inner radius (mm) | 12 | 9.5 | 12 | 15 | 15 |

Coordinates below are expressed in the anatomical frame. The RCM is its origin.

| Point or obstacle | Position (mm) |
| --- | --- |
| T0: initial tip position | `(0, 0, 75)` |
| T1 | `(3, 0, 75)` |
| T2 | `(0, 3, 75)` |
| O1 centre | `(3, 1.48, 74)` |
| O2 centre | `(1.48, -3, 74)` |

O1 and O2 are spherical obstacles of diameter 1 mm, stored in their respective scenes. Selecting a target does not reposition the obstacle.

Both planners sample tip positions and retain a joint configuration at each node. The local connector uses IK under the RCM constraint and checks sampled states along joint-space segments. Checks include:

- RCM error at most **0.1 mm** and tip-tracking error at most **0.5 mm**;
- shaft–wall and shaft–obstacle clearance at least **1 mm**;
- robot–environment collision checks and joint limits;
- maximum joint increment **0.5 degrees** and segment sampling resolution **0.1 degrees**.

The connector tip-command step is 0.1 mm. Lua settings use metres, radians and seconds. Instrument axial rotation is not independently constrained by the tracking task.

Validation is numerical and sampled. Self-collision is excluded, and no continuous collision-freedom guarantee is claimed. Replay is kinematic, with no validation of velocity, acceleration, forces or clinical safety. The corridor is synthetic and not patient-specific.

## 4. Run a simulation

### 4.1 Configure the local path

Open the selected `.ttt` scene in CoppeliaSim. In the simulation script attached to `/UR3`, adapt `projectRoot` to the local repository location, using forward slashes:

```lua
local projectRoot = 'C:/Users/YourName/Documents/GitHub/Robotic-endonasal-simulation'
```

The entry script loads the Lua modules from `projectRoot .. '/src/'`. Keep the embedded scene script consistent with `Scenes/Scripts/Script.lua`. Editing the source copy on disk alone does not establish that the script embedded in the `.ttt` scene was updated. Save a local scene copy after adapting its embedded path.

### 4.2 Select a scenario

Stop the simulation before editing `src/config.lua`. The target and execution mode are local variables near the beginning of `Config.create`. The other settings below are fields in its returned configuration table.

| Setting | CMP01: T2/O1 | CMP02: T1/O2 |
| --- | --- | --- |
| Scene | `endonasal_V0_posture_B_obstacle_O1.ttt` | `endonasal_V0_posture_B_obstacle_O2.ttt` |
| `targetName` | `'T2'` | `'T1'` |
| `runLabel` | `'V0_B_O1_' .. targetName` | `'V0_B_O2_' .. targetName` |
| `obstaclePath` | `'/Anatomy_V0/Obstacle_O1'` | `'/Anatomy_V0/Obstacle_O2'` |
| `obstacleEnabled` | `true` | `true` |

The target path is derived automatically as `'/Anatomy_V0/target_' .. targetName`. The `method` field is derived automatically from `executionMode`.

Keep both diagnostic flags disabled for normal planning and replay:

```lua
diagnosticGoalT1 = false,
diagnosticEndpoints = false,
```

Update `runNote` to describe the actual scenario, planner, seed and budget. This is descriptive metadata: editing it does not change planner behaviour. Likewise, `runLabel` names the output and does not select scene geometry.

### 4.3 Select a planner

Use the appropriate local variable at the start of `Config.create`:

```lua
local executionMode = 'rrt_replay'       -- RRT
-- or:
local executionMode = 'rrt_star_replay'  -- RRT*
```

Use only one of these declarations. Common campaign settings are:

```lua
rrtSeed = 999983,
rrtMaxIterations = 50,
profilePlanning = true,
```

For example, the supplied configuration selects T1/O2 with RRT and seed 999983:

```lua
runNote = 'CMP02; scenario=V0_B_O2_T1; target=T1; planner=RRT; seed=999983; budget=50',
```

For RRT*, change `executionMode` to `'rrt_star_replay'` and change the descriptive planner value in `runNote` to `RRT*`. Both methods use `rrtSeed` and `rrtMaxIterations`.

RRT stops at its first solution or budget exhaustion. RRT* continues to the full 50-iteration budget. Their measured total planning times therefore do not represent equal computation budgets.

### 4.4 Execute and export

1. Start the simulation and read the CoppeliaSim log. Planning can take several minutes; RRT does not necessarily print regular progress messages.
2. On success, the stored path is replayed with one waypoint per simulation step.
3. Stop the simulation normally after completion so that the cleanup callback exports the run.
4. Check the export-completion message and final status. A completed run reports `status=completed`.

A budget-exhausted run may suspend the simulation. Resume if needed and then stop normally to allow cleanup and export. Preserve these runs as failures: they are part of the experimental evidence. Do not replace failed trials with favourable seeds.

The `duration = 16` setting concerns prescribed motion, not the planned-path replay policy. A replay's displayed duration is not a dynamically validated trajectory duration.

### 4.5 Other execution modes and diagnostics

| Setting | Purpose |
| --- | --- |
| `executionMode = 'connector_replay'` | Attempt the direct local connection and replay it if accepted |
| `executionMode = 'prescribed_motion'` | Prescribed IK tracking, separate from the planner comparison |
| `diagnosticEndpoints = true` | Static endpoint checks, not a path test |
| `diagnosticGoalT1 = true` | Diagnostic of a T1 configuration, not an exhaustive reachability test |

Enable only the intended diagnostic and disable it again before a campaign run. Static diagnostic runs do not produce the normal planning/replay CSV exports. A valid endpoint does not imply that a direct connecting path is valid.

## 5. Exported data

Run directories are created under `Results/` with a timestamp and `runLabel`. Filenames use the scenario prefix, for example `V0_B_O2_T1_rrt_path.csv`.

| Filename suffix | Content |
| --- | --- |
| `_rrt_path.csv` / `_rrt_star_path.csv` | Calculated path |
| `_rrt_summary.csv` / `_rrt_star_summary.csv` | Planning summary and parameters |
| `_rrt_star_cost_history.csv` | Evolution of the best retained RRT* solution cost |
| `_states.csv` | Accepted replay states, joint values, actual/reference tip positions, errors, clearances and collision indicators |
| `_summary.csv` | Overall run outcome and summary measurements |

Direct-connector runs use `_connector_path.csv` and `_connector_summary.csv`. Rejected candidate states are not accepted replay states. A planning failure may export zero replay configurations; it must not be interpreted as a zero-length successful path.

## 6. Reproduce the MATLAB analyses

Keep the archived run directories in `Results/` and the scripts in `analysis/`. From the repository root in MATLAB, run:

```matlab
run('analysis/analyze_T2_O1.m')
run('analysis/analyze_T1_O2.m')
```

If MATLAB's Current Folder is already `analysis/`, use:

```matlab
run('analyze_T2_O1.m')
run('analyze_T1_O2.m')
```

The scripts locate the repository relative to their own file path. They use explicit run manifests: newly generated timestamped runs are not automatically added to the published campaigns. Missing archived directories cause an error; retain the original data when reproducing the reported figures.

Outputs are written to:

- `analysis/outputs/CMP01_T2_O1/`
- `analysis/outputs/CMP02_T1_O2/`

Each analysis writes `runs.csv`, `paired_comparison.csv`, `summary_statistics.csv`, `common_parameters.csv`, `analysis_notes.txt`, and the figures `path_lengths`, `planning_times`, and `cost_histories` in PDF and PNG formats. Rerunning an analysis replaces these derived outputs, not the raw experimental CSVs.

The scripts check the exported records and recompute path lengths from actual recorded tip positions. These consistency checks do not establish continuous validity between samples.

For the report, use the vector PDF figures. The two output directories contain identical base filenames: retain campaign-specific names when importing into Overleaf, such as `CMP01_T2_O1_path_lengths.pdf` and `CMP02_T1_O2_path_lengths.pdf`.

## 7. Experimental campaigns and results

Both campaigns use these ten paired seeds:

```text
104729, 130363, 155921, 196613, 262147,
314159, 524287, 654323, 786433, 999983
```

There are 20 attempts per campaign, one per planner and seed. Development seed 42 and preparatory direct-connector tests are excluded. CMP02 alternates execution order: RRT first for odd pairs and RRT* first for even pairs. CMP01 execution order is retained in its paired-comparison CSV, including the documented deviations for seeds 196613 and 314159.

| Campaign | Planner | Successes | Mean tip-path length ± sample SD (mm) | Mean planning time ± sample SD (s) |
| --- | --- | --- | --- | --- |
| CMP01: T2/O1 | RRT | 10/10 | 4.6666 ± 1.3390 | 116.816 ± 40.013 |
| CMP01: T2/O1 | RRT* | 10/10 | 4.2123 ± 1.1393 | 412.770 ± 32.208 |
| CMP02: T1/O2 | RRT | 9/10 | 3.8264 ± 0.3467 | 113.417 ± 70.419 |
| CMP02: T1/O2 | RRT* | 9/10 | 3.5115 ± 0.1426 | 425.192 ± 49.363 |

Length statistics include successful runs only. Time statistics include all ten attempts per planner and campaign. Both CMP02 planners exhausted the budget for seed 130363. These two failures are retained, with missing lengths rather than zero lengths.

RRT* returned shorter paths in eight CMP01 pairs and equal lengths in two. Among the nine jointly successful CMP02 pairs, four were shorter and five equal within numerical tolerance; the mean paired relative length reduction was 7.51%. This is distinct from the percentage reduction of the two mean lengths.

Planning times include geometric queries and profiling, and exclude replay and CSV export. No query or startup time is subtracted. Absolute times depend on the execution environment. The comparison demonstrates an observed path-length/computation-time trade-off under the chosen stopping rules, not general statistical superiority or a proof of optimality.

Both target and obstacle placement differ between campaigns; cross-campaign differences cannot be attributed to the obstacle alone. The tested static T1/O1 configuration was rejected for collision, which does not prove that every configuration reaching T1 is invalid.

## 8. Archive provenance

| Commit | Reference role |
| --- | --- |
| `dddf0799e85e61c08c5207f84a5c642b864a1825` | Earlier archived development snapshot referenced by CMP01 analysis |
| `59da78e2cc56827ecbad87abb7c739c986bd496c` | Pre-campaign CMP02 reference |
| `667554fd74832c707c5d0af9b7aa9421431709f1` | CMP02 scene/configuration and analysis archive |
| `3a1e18b639b71c76f3116350f8ca1a01a1121055` | Archive including the remaining CMP02 raw results |

These references identify archived snapshots, not verified execution hashes for every individual run. Later report, figure-formatting and documentation revisions may follow them. Record `git rev-parse HEAD` for the final delivered checkout; do not treat the table above as the latest repository state.

## 9. Troubleshooting

- **Cannot open a Lua module:** check the embedded UR3 script's `projectRoot` and the presence of `src/` beneath it.
- **Obstacle object not found or misleading run label:** ensure the open scene, `obstaclePath`, `runLabel`, target and `runNote` agree.
- **Only static diagnostic messages appear:** disable both diagnostic flags before planning.
- **MATLAB searches for `analysis/analysis`:** use the commands for the current working directory shown above.
- **MATLAB reports a missing run or summary key:** check the explicit run manifest and the corresponding exported files; do not bypass checks or silently substitute another run.
- **Old figures still appear in the report:** check the exact path and extension in `\includegraphics`, replace the referenced PDF or PNG, then recompile.

The report describes the modelling assumptions, methods, results and limitations. This README provides the practical steps for running the software and reproducing its archived analyses.
