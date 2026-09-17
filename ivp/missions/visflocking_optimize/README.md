# visflocking_optimize — reproducing Mezey et al., Supplementary Figure 3

Sweeps the vision-based flocking model's α0 (social acceleration) and β0 (social turning) over
a toroidal arena with 10 simulated boats, and renders the same summary metrics as
Supplementary Figure 3 of *Purely vision-based collective movement of robots* (Mezey, Bastien,
Zheng, McKee, Stoll, Hamann & Romanczuk, npj Robotics 3:11, 2025).

The reference implementation the paper itself ran is checked out in this workspace at
`ABM/abm/projects/visual_flocking/` (SI ref. [2], `scioip34/ABM`); when a modelling detail is
ambiguous, that code — not the SI text — is the arbiter.

## Scaling: how the paper's units map onto this mission

The paper's model is written in pixels and simulation timesteps: agents have radius
R_A = 5.5 px in a 900 px arena, move v0 = 1 px per timestep, and γ / α0 / β0 are rates *per
timestep* (`vf_agent.update_agent_position`: `orientation += dphi`, `velocity += dv`,
`position += velocity`). Two ratios have to be preserved:

| quantity | paper | this mission |
|---|---|---|
| arena / agent radius | 900 / 5.5 = 164 | 90.2 m / 0.55 m = 164 |
| travel per model timestep | 1 px = 0.18 R_A | 0.3 m/s × 1/3 s = 0.1 m = 0.18 R_A |
| retina | 320 bins over 2π (1.125°/bin) | `resolution` scaled with FOV, same °/bin |
| vision range | blob must span ≥2 bins ⇒ 51 R_A | same rule ⇒ 28 m = 51 R_A |

So `meta_vehicle.bhv` sets `v0 = 0.3` (m/s) and `time_scale = 3` (model timesteps per second),
and α0/β0/α1/β1/γ keep the paper's literal values — the heatmap axes are directly comparable
with Supp. Fig. 3. The paper's 20000 timesteps are 6667 s of sim time.

## Latency budget (the thing that breaks this mission)

The flocking state is sensitive to how *stale* the visual field is, measured in model
timesteps. Re-running the paper's own model with an artificial perception lag:

| perception lag | 0–2 timesteps | 3 timesteps | 5 timesteps |
|---|---|---|---|
| polarization at α0 = 0.5, β0 = 0.1 | 0.79–0.88 | 0.74 | **0.39** |

The MOOS path — vehicle `pShare` → shoreside `pEchoVar` → `pSimVisionServer` → shoreside
`pShare` → helm — therefore has to stay well under one model timestep. That is why this mission
runs those apps at 20 Hz (they were at 4 Hz), `nav_modulo = 1` in uSimMarineV23, and
`MOOSTimeWarp = 10`. **Raising the warp is not free**: at warp 20 this machine only sustains
7.7 Hz in the helm and 15 Hz in the vision server, the effective lag grows past the cliff above,
and the flocking anchor collapses from P = 0.82 to P = 0.28. If you raise it, re-measure the
achieved rates from an `.alog` and re-run the F anchor before trusting a sweep.

## Running

```bash
./launch.sh <a0> <a1> <b0> <b1> <gam> <fov_deg> <run_id> ["x,y,hdg;x,y,hdg;..."]
./optimize.py --fov 100 --random            # one Supp. Fig. 3 column
./analyze_results.py                        # heatmaps
```

Environment switches understood by `launch.sh`:

- `LOGGING=yes` — start `pLogger` on the shoreside (whole-fleet `NODE_REPORT`s, plus
  `VPF_ALPHA_001/2`) and on each vehicle (`DEBUG_*`, `DESIRED_*` vs `NAV_*`). Off for sweeps.
- `DYNAMICS=boat` — rudder-steered hull instead of the near-holonomic baseline (see below).
- `VEHICLE_COUNT=2` — small diagnostic runs.

`plot_trajectories.py <MOOSLog dir> -o out.png` renders orientation-coloured trajectories,
unwrapped across the torus — the view the paper's movement-pattern labels (X, LeFo, frLeFo, F,
frF, L) are assigned from. No combination of the summary metrics separates LeFo from frLeFo.

## Verified anchors (FOV 100 %)

Each MOOS run below is a single `./launch.sh` run; the reference column is the paper's model
run at the same parameters (10 agents, 900 px torus, 20000 timesteps).

| anchor | α0, β0 | MOOS: P / D | reference: P / D |
|---|---|---|---|
| Unordered (X) | 0, 0 | 0.29 / 61 R_A | 0.35 / 62.5 R_A |
| Flocking (F) | 0.5, 0.1 | 0.82 / 14.4 R_A | 0.83 / 13.6 R_A |
| Swarming (S) | 0.5, 2.0 | 0.25 / 12.9 R_A | 0.28 / 12.8 R_A |

## Baseline vs. real boat dynamics

`DYNAMICS=baseline` (default) makes the hull near-holonomic — `holonomic_turn = true`, no
turn-rate clip, no deceleration limit, no turn loss. The paper's agents are holonomic points
that turn tens of deg/s while braking, which a rudder-steered hull cannot do (uSimMarineV23
clips the turn rate at 100 deg/s and scales it by both speed and thrust, so turn authority
collapses exactly when the model commands a slow-down). This is the configuration for asking
whether the *model* reproduces the paper.

`DYNAMICS=boat` restores the hull as configured for the real boats. Measured at the F anchor:

| | polarization | mean distance | largest cluster | overlap |
|---|---|---|---|---|
| baseline | 0.75 | 8.0 m | 8.6 / 10 | 0.06 |
| boat | 0.29 | 12.3 m | 5.2 / 10 | 0.14 |

i.e. with this hull model the flocking state is not reachable at the paper's parameters — worth
knowing before expecting Supp. Fig. 3 behaviour from the real boats.
