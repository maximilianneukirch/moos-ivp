#!/usr/bin/env python3
"""
optimize.py -- reproduce Mezey et al., Supplementary Fig. 3/4 with the
MOOS/IvP implementation of BHV_VisFlocking.

The paper's Fig. 3/4 sweep two parameters, alpha0 (social
acceleration/deceleration strength, BHV_VisFlocking's `a0`) and beta0
(social turning strength, `b0`), across four fields of view (100%/75%/
50%/25% of 360 degrees -- one column per FOV in both figures), while
holding everything else fixed at the values in Supplementary Table 1:

    alpha1 = beta1 = 0.09   (equilibrium front-back / left-right distance)
    gamma  = 0.1            (self-propulsion preference factor)
    v0     = 1.0            (preferred individual speed; fixed in the .bhv)
    N      = 10 agents

This script mirrors that: A1_SPACE/B1_SPACE/GAM_SPACE default to single
paper values, and only A0_SPACE/B0_SPACE x FOV_SPACE are swept. You *can*
widen any of the spaces below to run a fuller hyperparameter search instead
of a 1:1 figure reproduction -- the grid is just the product of all five
lists (a0 x a1 x b0 x b1 x gam), repeated once per FOV.

Each combo is one MOOS simulation launched via ./launch.sh, given time to
run, then killed and cleaned up before the next combo starts (the MOOS apps
involved don't tolerate a second overlapping instance sharing ports).
uFlockEvaluator logs a running-average row to flock_evaluation.csv on every
Iterate() tick once the fleet is actually deployed (it ignores ticks before
DEPLOY_ALL fires, so the pre-deploy stationary phase isn't averaged in);
this script tags each run with a unique RunID (passed through to
pFlockEvaluator's RUN_ID param), truncates flock_evaluation.csv before every
run (so it only ever holds *this* run's rows -- otherwise it grows unbounded
over a multi-hour, multi-FOV sweep), and pulls out that run's *last* row --
i.e. the average over the run's post-deploy lifetime.

Each FOV's results go to their own file, hyperparameter_results_fov<PCT>.csv
(PCT in 100/75/50/25), so the four Fig. 3/4 columns don't get mixed
together and analyze_results.py can render them as separate heatmap
columns.

Usage:
    ./optimize.py                      # full default grid x 4 FOVs (see below)
    ./optimize.py --quick              # tiny 2x2 grid @ 2 FOVs, ~4 min, sanity check
    ./optimize.py --run-seconds 90     # longer settling time per run
    ./optimize.py --resume             # skip combos already in their FOV's results CSV
    ./optimize.py --fov 100,25         # only the two extreme FOV columns
    ./optimize.py --a0 0,0.5,1,2 --b0 0,1,4   # custom a0/b0 grid
    ./optimize.py --random                    # paper-style random start poses, fresh draw per run
    ./optimize.py --random --seed 42           # ...but the *same* draw for every run (see --seed)

Expect the full default grid to take *hours* (see the printed ETA before it
starts). Run --quick first to confirm the pipeline works end to end.
"""

import argparse
import itertools
import os
import random
import subprocess
import time

import pandas as pd

MISSION_DIR = os.path.dirname(os.path.abspath(__file__))

# --- Paper (Supplementary Table 1) values, held fixed by default ---
A1_SPACE = [0.09]
B1_SPACE = [0.09]
GAM_SPACE = [0.1]

# --- The two axes actually swept in Supp. Fig. 3/4 ---
# Denser near 0 like the paper's own axis ticks (0, 0.05, 0.5, 1.25, 4 for
# alpha0; 0, 0.01, 0.05, 0.1, 0.5, ... for beta0). Widen/narrow freely --
# more points = a smoother heatmap but a longer sweep.
A0_SPACE = [0.0, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 0.75, 1.0, 1.25, 2.0, 4.0]
B0_SPACE = [0.0, 0.01, 0.02, 0.03, 0.05, 0.1, 0.2, 0.5, 0.75, 1.0, 1.25, 2.0, 4.0]

# --- The four FOV columns in Supp. Fig. 3/4, as % of 360 degrees ---
FOV_SPACE_PCT = [100, 75, 50, 25]

# Must match launch.sh's VEHICLE_COUNT and meta_shoreside.moos's ARENA_WIDTH
# (the torus's full wrap period, i.e. 2x the wormhole half-width in
# meta_vehicle.moos) -- both are currently fixed, not CLI-configurable.
#
# 90.2 m = 164 x the 0.55 m agent radius, the same arena-to-agent ratio the
# paper uses (900 px arena, 5.5 px agents). That ratio -- not the absolute
# size -- is what the vision-based model actually runs on.
VEHICLE_COUNT = 10
ARENA_WIDTH = 90.2

EVAL_CSV = os.path.join(MISSION_DIR, "flock_evaluation.csv")

KILL_PROCS = (
    "pAntler MOOSDB pMarineViewer pShare uSimMarineV23 pHelmIvP "
    "pMarinePIDV22 pNodeReporter pSimVisionServer uProcessWatch pLogger "
    "uTimerScript uFlockEvaluator"
)


def fov_deg(pct):
    return pct / 100.0 * 360.0


def results_csv_for(fov_pct):
    return os.path.join(MISSION_DIR, f"hyperparameter_results_fov{fov_pct}.csv")


def cleanup():
    """Kill every process the mission can spawn and wait for ports to free up.

    Mirrors launch.sh's own pre-cleanup. Done twice with `ktm` (moos-ivp's
    "kill the MOOS" helper) interleaved, same as the previous version of
    this script -- pAntler's children don't always die on the first signal.
    """
    os.system(f"killall -q -9 {KILL_PROCS}")
    time.sleep(1)
    os.system("ktm")
    time.sleep(2)
    os.system(f"killall -q -9 {KILL_PROCS}")
    time.sleep(1)
    os.system("ktm")
    time.sleep(2)


def run_id_for(fov_pct, a0, a1, b0, b1, gam, poses_tag="fixed"):
    run_id = f"fov_{fov_pct}_a0_{a0}_a1_{a1}_b0_{b0}_b1_{b1}_gam_{gam}"
    if poses_tag != "fixed":
        run_id += f"_poses_{poses_tag}"
    return run_id


def random_start_poses(n, arena_width, rng):
    """n (x, y, heading_deg) triples, paper-style: positions uniform in a
    square centered on the arena origin with side arena_width/3 (the
    paper's "central third of the arena"), headings uniform in [-180, 180)
    (paper: orientation uniform in [-pi, pi))."""
    half_side = arena_width / 3.0 / 2.0
    return [
        (rng.uniform(-half_side, half_side), rng.uniform(-half_side, half_side), rng.uniform(-180.0, 180.0))
        for _ in range(n)
    ]


def already_done(fov_pct, run_id):
    results_csv = results_csv_for(fov_pct)
    if not os.path.exists(results_csv):
        return False
    try:
        df = pd.read_csv(results_csv)
    except pd.errors.EmptyDataError:
        return False
    return "RunID" in df.columns and (df["RunID"] == run_id).any()


def run_one(fov_pct, a0, a1, b0, b1, gam, run_seconds, use_random_poses=False, seed=None):
    """use_random_poses: paper-style random start poses (--random) instead of
    launch.sh's default fixed staggered-line start.

    seed: if given, a *fresh* random.Random(seed) is created for this run
    alone, so every run in the sweep draws the identical first
    VEHICLE_COUNT poses from the same deterministic sequence -- i.e. the
    same seed always produces the same start layout, for every combo, in
    every invocation (including --resume, where combos may be skipped in a
    different order than a from-scratch run). This is deliberate: it's
    what makes a0/b0/fov comparisons apples-to-apples under --random.
    (Previously a single random.Random was created once in main() and
    reused/advanced across the whole sweep, so each combo silently drew a
    *different* layout, and which layout depended on execution order --
    same seed did not mean same poses.)
    If seed is None, a fresh unseeded random.Random() is created per run
    instead, so each run still gets its own independent random draw."""
    poses_tag = "fixed" if not use_random_poses else "random"
    run_id = run_id_for(fov_pct, a0, a1, b0, b1, gam, poses_tag)

    cleanup()  # make sure nothing from a previous (possibly killed) run lingers

    # Start every run with a clean slate: flock_evaluation.csv is append-only
    # (uFlockEvaluator opens it with ios::app), so without this it would
    # grow across the *entire* sweep (all FOVs x all combos) instead of
    # holding just this run's rows.
    if os.path.exists(EVAL_CSV):
        os.remove(EVAL_CSV)

    start_poses_arg = ""
    if use_random_poses:
        rng = random.Random(seed)
        poses = random_start_poses(VEHICLE_COUNT, ARENA_WIDTH, rng)
        start_poses_arg = ";".join(f"{x},{y},{h}" for x, y, h in poses)

    subprocess.Popen(
        ["./launch.sh", str(a0), str(a1), str(b0), str(b1), str(gam), str(fov_deg(fov_pct)), run_id,
         start_poses_arg],
        cwd=MISSION_DIR,
    )
    time.sleep(run_seconds)

    cleanup()

    if not os.path.exists(EVAL_CSV):
        print("  !! flock_evaluation.csv not found after run -- skipping")
        return None

    try:
        df = pd.read_csv(EVAL_CSV)
    except pd.errors.EmptyDataError:
        print("  !! flock_evaluation.csv has no data yet -- skipping")
        return None

    run_rows = df[df["RunID"] == run_id]
    if run_rows.empty:
        print(f"  !! no rows tagged RunID={run_id} -- simulation produced no data")
        return None

    last = run_rows.iloc[-1]
    result = {
        "fov_pct": fov_pct,
        "fov_deg": fov_deg(fov_pct),
        "a0": a0,
        "a1": a1,
        "b0": b0,
        "b1": b1,
        "gam": gam,
        "poses": poses_tag,
        "RunID": run_id,
        "PolarizationOrder": float(last["PolarizationOrder"]),
        "MeanDistance": float(last["MeanDistance"]),
        "MaxClusterSize": float(last["MaxClusterSize"]),  # a run-average, not an integer
        "AreaToCircleRatio": float(last["AreaToCircleRatio"]),
        "OverlapRatio": float(last["OverlapRatio"]),
        "Iterations": int(last["Iterations"]),
    }
    print(
        f"  -> Pol={result['PolarizationOrder']:.2f}  "
        f"Dist={result['MeanDistance']:.2f}  "
        f"Overlap={result['OverlapRatio']:.2f}  "
        f"(n={result['Iterations']} ticks)"
    )
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--a0", type=str, default=None, help="comma-separated alpha0 values, overrides A0_SPACE")
    ap.add_argument("--b0", type=str, default=None, help="comma-separated beta0 values, overrides B0_SPACE")
    ap.add_argument("--a1", type=str, default=None, help="comma-separated alpha1 values, overrides A1_SPACE")
    ap.add_argument("--b1", type=str, default=None, help="comma-separated beta1 values, overrides B1_SPACE")
    ap.add_argument("--gam", type=str, default=None, help="comma-separated gamma values, overrides GAM_SPACE")
    ap.add_argument("--fov", type=str, default=None,
                     help="comma-separated FOV percentages (of 360deg), overrides FOV_SPACE_PCT "
                          "(default: 100,75,50,25)")
    ap.add_argument("--run-seconds", type=int, default=720,
                     help="real-world seconds per combo. One model timestep is "
                          "1/3 s (meta_vehicle.bhv: v0 = 0.3, time_scale = 3), so the "
                          "paper's 20000 timesteps are 6667 s of sim time = 667 real s "
                          "at MOOSTimeWarp = 10, plus the 300 s (30 real s) pre-deploy "
                          "settle. uFlockEvaluator's WARMUP_SECONDS discards the first "
                          "2000 s, so shortening this eats into the measured window; "
                          "--run-seconds 400 (10000 timesteps) roughly halves sweep cost "
                          "and still lands on the same anchors. Do NOT raise MOOSTimeWarp "
                          "to buy speed without re-checking the achieved tick rates and "
                          "the F anchor -- see the note in meta_vehicle.moos.")
    ap.add_argument("--quick", action="store_true",
                     help="tiny 2x2 a0/b0 grid at 2 FOVs with a short run, to sanity-check the pipeline")
    ap.add_argument("--resume", action="store_true",
                     help="skip combos whose RunID is already in that FOV's results CSV")
    ap.add_argument("--random", action="store_true",
                     help="paper-style random start poses (positions uniform in the central "
                          "third of the arena, headings uniform in [-180,180)) instead of the "
                          "fixed staggered-line start, to check whether the fixed start biases "
                          "the sweep")
    ap.add_argument("--seed", type=int, default=None,
                     help="freeze --random's start poses to the same seeded draw for every run "
                          "in the sweep (so a0/b0/fov comparisons are apples-to-apples); default "
                          "is a fresh, independently-random draw per run")
    args = ap.parse_args()

    def parse_list(s, default, cast=float):
        return [cast(x) for x in s.split(",")] if s else default

    a0_space = parse_list(args.a0, A0_SPACE)
    b0_space = parse_list(args.b0, B0_SPACE)
    a1_space = parse_list(args.a1, A1_SPACE)
    b1_space = parse_list(args.b1, B1_SPACE)
    gam_space = parse_list(args.gam, GAM_SPACE)
    fov_space = parse_list(args.fov, FOV_SPACE_PCT, cast=int)
    run_seconds = args.run_seconds

    if args.quick:
        a0_space, b0_space = [0.0, 1.0], [0.0, 1.0]
        a1_space, b1_space, gam_space = [0.09], [0.09], [0.1]
        fov_space = [100, 25]
        run_seconds = 30  # >15s deploy-delay headroom (see meta_shoreside.moos) + some flocking time

    combinations = list(itertools.product(a0_space, a1_space, b0_space, b1_space, gam_space))
    n_per_fov = len(combinations)
    n = n_per_fov * len(fov_space)
    cleanup_overhead = 6  # rough seconds per run spent in cleanup()
    eta_min = n * (run_seconds + cleanup_overhead) / 60.0

    print(f"Sweeping {n_per_fov} a0/b0/a1/b1/gam combinations x {len(fov_space)} "
          f"FOVs ({fov_space}) = {n} runs, ~{run_seconds}s each (+cleanup) "
          f"-> ETA ~{eta_min:.0f} min ({eta_min / 60:.1f} h)")
    if not args.quick and n > 20:
        print("This is a large sweep -- consider `--quick` first to confirm "
              "everything works, or `--resume` if you're continuing a previous run.")

    os.chdir(MISSION_DIR)

    poses_tag = "random" if args.random else "fixed"

    grand_total_done = 0
    for fov_pct in fov_space:
        results_csv = results_csv_for(fov_pct)
        all_results = []
        if args.resume and os.path.exists(results_csv):
            try:
                all_results = pd.read_csv(results_csv).to_dict("records")
            except pd.errors.EmptyDataError:
                pass

        print(f"\n=== FOV {fov_pct}% ({fov_deg(fov_pct):.0f} deg) -> {os.path.basename(results_csv)} ===")

        for idx, (a0, a1, b0, b1, gam) in enumerate(combinations):
            run_id = run_id_for(fov_pct, a0, a1, b0, b1, gam, poses_tag)
            if args.resume and already_done(fov_pct, run_id):
                print(f"[{idx + 1}/{n_per_fov}] skip (already done): {run_id}")
                continue

            print(f"\n--- FOV {fov_pct}% run {idx + 1}/{n_per_fov} --- a0={a0} a1={a1} b0={b0} b1={b1} gam={gam}")
            result = run_one(fov_pct, a0, a1, b0, b1, gam, run_seconds,
                              use_random_poses=args.random, seed=args.seed)
            if result is not None:
                all_results.append(result)
                # Persist incrementally so a crash/interrupt mid-sweep doesn't
                # lose everything before it.
                pd.DataFrame(all_results).to_csv(results_csv, index=False)

        print(f"--- FOV {fov_pct}%: {len(all_results)}/{n_per_fov} combos produced data "
              f"-> {results_csv} ---")
        grand_total_done += len(all_results)

    print(f"\n=== Sweep complete: {grand_total_done}/{n} runs produced data ===")
    print("Run analyze_results.py to render the Fig. 3/4-style heatmaps (one PNG per FOV).")


if __name__ == "__main__":
    main()
