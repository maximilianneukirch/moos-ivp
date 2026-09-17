#!/usr/bin/env python3
"""Plot vision-flocking trajectories the way Mezey et al. do (Supp. Fig. 2d/3):
one panel per run,每 agent's track coloured by its heading, unwrapped across the
torus so wraparound does not streak lines across the arena.

The movement patterns the paper names -- Unordered (X), Leader-Follower (LeFo),
fragmented Leader-Follower (frLeFo), Flocking (F), fragmented Flocking (frF),
Lines (L) -- are assigned by eye from exactly this view; no combination of the
summary metrics separates LeFo from frLeFo.

Usage:
    ./plot_trajectories.py <MOOSLog dir or .alog> [more ...] -o out.png
Reads NODE_REPORT (shoreside log sees the whole fleet; a vehicle log works too
but only carries the boats it heard about).
"""
import argparse, glob, math, os, sys

def read_alog(path):
    tracks = {}
    with open(path, errors="ignore") as fh:
        for line in fh:
            if line.startswith("%"):
                continue
            parts = line.split(None, 3)
            if len(parts) < 4 or parts[1] != "NODE_REPORT":
                continue
            t = float(parts[0])
            rec = dict(kv.split("=", 1) for kv in parts[3].strip().split(",") if "=" in kv)
            if "NAME" not in rec:
                continue
            tracks.setdefault(rec["NAME"], []).append(
                (t, float(rec["X"]), float(rec["Y"]), float(rec["HDG"])))
    for v in tracks.values():
        v.sort()
    return tracks

def unwrap(track, period):
    """Break the track wherever it jumps more than half the wrap period, so a
    wraparound is a new line segment instead of a line across the arena."""
    segs, cur = [], []
    for i, (t, x, y, h) in enumerate(track):
        if cur:
            _, px, py, _ = track[i - 1]
            if abs(x - px) > period / 2 or abs(y - py) > period / 2:
                segs.append(cur); cur = []
        cur.append((x, y, h))
    if cur:
        segs.append(cur)
    return segs

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="MOOSLog dirs or .alog files")
    ap.add_argument("-o", "--out", default="trajectories.png")
    ap.add_argument("--arena", type=float, default=90.2, help="torus wrap period (m)")
    ap.add_argument("--start", type=float, default=None, help="skip data before this log time (s)")
    args = ap.parse_args()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("matplotlib is required: pip install matplotlib")

    paths = []
    for spec in args.logs:
        paths += sorted(glob.glob(os.path.join(spec, "*.alog"))) if os.path.isdir(spec) else [spec]

    n = len(paths)
    cols = min(n, 3); rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 5 * rows), squeeze=False)
    half = args.arena / 2

    for ax, path in zip([a for r in axes for a in r], paths):
        tracks = read_alog(path)
        for name, tr in sorted(tracks.items()):
            if args.start is not None:
                tr = [p for p in tr if p[0] >= args.start]
            for seg in unwrap(tr, args.arena):
                xs = [p[0] for p in seg]; ys = [p[1] for p in seg]; hs = [p[2] for p in seg]
                # colour each step by heading with a cyclic colormap
                for i in range(len(seg) - 1):
                    ax.plot(xs[i:i+2], ys[i:i+2], lw=0.7,
                            color=plt.cm.twilight((hs[i] % 360) / 360.0))
            if tr:
                ax.plot(tr[-1][1], tr[-1][2], "o", ms=4, color="k")
        ax.set_xlim(-half, half); ax.set_ylim(-half, half)
        ax.set_aspect("equal")
        ax.set_title(os.path.basename(path.rstrip("/")), fontsize=8)
    for ax in [a for r in axes for a in r][n:]:
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print("wrote", args.out)

if __name__ == "__main__":
    main()
