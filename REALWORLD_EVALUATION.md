# Planned Real-World Evaluation

Snapshot: **2026-08-27** · Status: **protocol template; no formal results**

This page reserves the publication structure for the first MemNav real-world
campaign. It follows the
[TopoFocus Real-World](https://github.com/AlanZhu2006/topofocus_realworld)
presentation pattern: one frozen
scene definition, five formal rollouts, per-run metrics, dual-view media and a
machine-readable manifest. Empty values are deliberately shown as `—`; they do
not mean zero, failure or missing-at-random data.

The initial campaign evaluates the deployed **CEC-certified bearing + frozen
NavDP** route in four static scenes with five repeats per scene: 20 formal runs
in total. A later baseline comparison must duplicate the complete 20-run
campaign for each method rather than pooling methods inside these rows.

## Metric Contract

For run `i`:

```text
SR = sum(S_i) / N
SPL_i = S_i * L_i / max(L_i, P_i)
SPL = sum(SPL_i) / N
```

- `S_i` is the independently adjudicated binary navigation success.
- `L_i` is the predeclared shortest feasible path for that scene.
- `P_i` is the independently measured physical path travelled by the Go2.
- A failure contributes zero SPL.
- A dash must not be replaced until the run has a finalized capture manifest,
  an independent success record and a valid path measurement.

The exact physical success threshold and measurement system remain pending the
arrival-calibration gate in `CURRENT_STATUS.md`. Until that gate is frozen,
these tables are a registration template and cannot support an autonomous
arrival, SR or SPL claim. If an external evaluator terminates the robot, the
result must be described as **navigation with independent evaluator
termination**, not autonomous policy STOP.

## Evaluation Settings

| Scene | Target | Navigation setting | Dataset ID | Goal SHA-256 | Shortest feasible path `L` |
| --- | --- | --- | --- | --- | ---: |
| Scene 01 | `TBD` | `TBD` | `TBD` | `TBD` | `—` |
| Scene 02 | `TBD` | `TBD` | `TBD` | `TBD` | `—` |
| Scene 03 | `TBD` | `TBD` | `TBD` | `TBD` | `—` |
| Scene 04 | `TBD` | `TBD` | `TBD` | `TBD` | `—` |

All runs use the frozen formal control profile (`0.30 m/s` maximum linear
speed), one exact goal JPEG per scene, one sealed survey dataset per scene and
the `audit` evidence profile. Scene layouts, starts, goal poses, path budgets,
time budgets and trial order must be frozen before Formal 01 begins.

## Campaign Summary

| Scene | Planned trials | Completed | Successes | SR | Mean SPL |
| --- | ---: | ---: | ---: | ---: | ---: |
| Scene 01 | `5` | `0` | `—` | `—` | `—` |
| Scene 02 | `5` | `0` | `—` | `—` | `—` |
| Scene 03 | `5` | `0` | `—` | `—` | `—` |
| Scene 04 | `5` | `0` | `—` | `—` | `—` |
| **Overall** | **`20`** | **`0`** | **`—`** | **`—`** | **`—`** |

## Scene 01 · TBD

| Trials | Completed | Successes | SR | Mean SPL |
| ---: | ---: | ---: | ---: | ---: |
| `5` | `0` | `—` | `—` | `—` |

### Planned Rollouts

<table width="100%">
  <tr>
    <td width="50%" align="center"><strong>Formal 01 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
    <td width="50%" align="center"><strong>Formal 02 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
  </tr>
  <tr>
    <td width="50%" align="center"><strong>Formal 03 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
    <td width="50%" align="center"><strong>Formal 04 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
  </tr>
  <tr>
    <td colspan="2" align="center"><strong>Formal 05 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
  </tr>
</table>

### Per-Run Metrics

| Run | Result | Actual path `P` | Success basis | SPL | Evidence manifest |
| --- | --- | ---: | --- | ---: | --- |
| Formal 01 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 02 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 03 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 04 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 05 | `PENDING` | `—` | `—` | `—` | `—` |

## Scene 02 · TBD

| Trials | Completed | Successes | SR | Mean SPL |
| ---: | ---: | ---: | ---: | ---: |
| `5` | `0` | `—` | `—` | `—` |

### Planned Rollouts

<table width="100%">
  <tr>
    <td width="50%" align="center"><strong>Formal 01 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
    <td width="50%" align="center"><strong>Formal 02 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
  </tr>
  <tr>
    <td width="50%" align="center"><strong>Formal 03 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
    <td width="50%" align="center"><strong>Formal 04 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
  </tr>
  <tr>
    <td colspan="2" align="center"><strong>Formal 05 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
  </tr>
</table>

### Per-Run Metrics

| Run | Result | Actual path `P` | Success basis | SPL | Evidence manifest |
| --- | --- | ---: | --- | ---: | --- |
| Formal 01 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 02 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 03 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 04 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 05 | `PENDING` | `—` | `—` | `—` | `—` |

## Scene 03 · TBD

| Trials | Completed | Successes | SR | Mean SPL |
| ---: | ---: | ---: | ---: | ---: |
| `5` | `0` | `—` | `—` | `—` |

### Planned Rollouts

<table width="100%">
  <tr>
    <td width="50%" align="center"><strong>Formal 01 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
    <td width="50%" align="center"><strong>Formal 02 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
  </tr>
  <tr>
    <td width="50%" align="center"><strong>Formal 03 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
    <td width="50%" align="center"><strong>Formal 04 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
  </tr>
  <tr>
    <td colspan="2" align="center"><strong>Formal 05 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
  </tr>
</table>

### Per-Run Metrics

| Run | Result | Actual path `P` | Success basis | SPL | Evidence manifest |
| --- | --- | ---: | --- | ---: | --- |
| Formal 01 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 02 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 03 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 04 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 05 | `PENDING` | `—` | `—` | `—` | `—` |

## Scene 04 · TBD

| Trials | Completed | Successes | SR | Mean SPL |
| ---: | ---: | ---: | ---: | ---: |
| `5` | `0` | `—` | `—` | `—` |

### Planned Rollouts

<table width="100%">
  <tr>
    <td width="50%" align="center"><strong>Formal 01 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
    <td width="50%" align="center"><strong>Formal 02 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
  </tr>
  <tr>
    <td width="50%" align="center"><strong>Formal 03 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
    <td width="50%" align="center"><strong>Formal 04 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
  </tr>
  <tr>
    <td colspan="2" align="center"><strong>Formal 05 · PENDING</strong><br><small>Third view: pending</small><br><small>Dashboard: pending</small><br><small>Metrics: —</small></td>
  </tr>
</table>

### Per-Run Metrics

| Run | Result | Actual path `P` | Success basis | SPL | Evidence manifest |
| --- | --- | ---: | --- | ---: | --- |
| Formal 01 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 02 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 03 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 04 | `PENDING` | `—` | `—` | `—` | `—` |
| Formal 05 | `PENDING` | `—` | `—` | `—` | `—` |

## Evidence Publication Rule

Each populated run must link all of the following under one run ID:

1. finalized capture manifest and its SHA-256;
2. sealed survey dataset ID and manifest SHA-256;
3. Jetson and RTX Git commits;
4. independent success/path adjudication;
5. actual path `P`, scene shortest feasible path `L` and computed SPL;
6. third-view and RViz dashboard browser derivatives;
7. failure or intervention attribution when `S_i=0`.

Runtime evidence remains ignored by Git. Only independently reviewed manifests,
audit summaries and browser-ready media derivatives should be committed. The
machine-readable campaign template is
[`manifests/realworld_evaluation_plan_v1.json`](manifests/realworld_evaluation_plan_v1.json).
