#!/usr/bin/env python3
"""
analyze_results.py -- render Mezey et al. Supp. Fig. 3/4-style alpha0 x
beta0 heatmaps from optimize.py's per-FOV hyperparameter_results_fov<PCT>.csv
files (PCT in 100/75/50/25).

Produces:
  - one heatmap PNG per FOV found, e.g. 01_heatmaps_fov100.png
  - if more than one FOV is present: a combined PNG with all found FOVs as
    columns (paper column order 100% -> 25%), matching the paper's Fig. 3/4
    layout, 01_figure3_4_heatmaps.png

Styling intentionally mimics the paper's own panels: square heatmap blocks
with a plain border (no per-cell gridlines), and a bordered colorbar with
its tick numbers on the *left* (between the heatmap and the bar) and the
metric's symbol/name to the right. Metric symbols use matplotlib's mathtext
(the `$...$` bits below) rather than a real LaTeX install (`text.usetex`) --
this sandbox's LaTeX toolchain is missing `dvipng`, which usetex requires,
and mathtext needs no external dependency at all to render sub/superscripts
and Greek letters correctly.
"""
import argparse
import csv
import glob
import math
import os
import re

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.colors import ListedColormap
from matplotlib.ticker import MaxNLocator, ScalarFormatter

# (csv column, mathtext symbol, description, fixed vmin, fixed vmax) --
# symbol is drawn large above a thin rule, description below it, to the
# right of each row's colorbar. vmin/vmax fixed to sensible, round values
# (matching what the paper itself uses / the known upper bound -- N=10
# agents for MaxClusterSize) rather than the raw data range; None/None
# means "derive a nice range from the data" (see nice_bounds_and_ticks()).
METRICS = [
    ('PolarizationOrder', r'$P$', 'Polarization\nOrder', 0.0, 1.0),
    ('MeanDistance', r'$D$', 'Mean\nInter-individual\nDistance', None, None),
    ('MaxClusterSize', r'$N_{clus}^{max}$', 'Size of\nLargest cluster', 0.0, 10.0),
    ('AreaToCircleRatio', r'$RCA$', 'Area-to-Circle\nRatio', None, None),
    ('OverlapRatio', r'$R_o^{sim}$', 'Time Ratio\nIn Overlap', None, None),
]

BORDER_COLOR = 'black'
BORDER_WIDTH = 0.9


def nice_ceil(x):
    """Round x up to a "nice" number: 1/2/2.5/5/10 times a power of ten."""
    if x <= 0:
        return 1.0
    exp = math.floor(math.log10(x))
    frac = x / 10 ** exp
    for nice in (1, 2, 2.5, 5, 10):
        if frac <= nice + 1e-9:
            return nice * 10 ** exp
    return 10 * 10 ** exp  # unreachable, but keeps the type checker happy


def nice_bounds_and_ticks(data_min, data_max, fixed_vmin, fixed_vmax):
    """Pick a colorbar (vmin, vmax, ticks) that always labels both ends.

    vmin/vmax are either the caller's fixed, already-round values, or 0 and
    a nice_ceil() of the data max. Ticks come from a standard nice-number
    locator, with the two endpoints forced in if the locator didn't already
    land on them -- matplotlib's default tick placement can otherwise leave
    the top/bottom of the bar unlabeled (e.g. ticks at 0/50/100/150 on a bar
    that actually runs to 180).
    """
    if fixed_vmin is not None and fixed_vmax is not None:
        vmin, vmax = fixed_vmin, fixed_vmax
    else:
        #vmin, vmax = 0.0, nice_ceil(data_max if data_max > 0 else 1.0)
        vmin, vmax = 0.0, (data_max if data_max > 0 else 1.0)

    ticks = list(MaxNLocator(nbins=5, steps=[1, 2, 2.5, 5, 10]).tick_values(vmin, vmax))
    ticks = [t for t in ticks if vmin - 1e-9 <= t <= vmax + 1e-9]
    if not ticks or abs(ticks[0] - vmin) > 1e-9:
        ticks.insert(0, vmin)
    if abs(ticks[-1] - vmax) > 1e-9:
        ticks.append(vmax)
    return vmin, vmax, ticks


def load_fov_results():
    """Find hyperparameter_results_fov<PCT>.csv files, in paper column order (100 -> 25)."""
    results = {}
    for path in glob.glob('hyperparameter_results_fov*.csv'):
        m = re.search(r'fov(\d+)\.csv$', path)
        if not m:
            continue
        pct = int(m.group(1))
        df = pd.read_csv(path)
        if df.empty:
            continue
        # a1/b1/gam are (by default) fixed to single paper values, but average
        # over them anyway in case a wider sweep was run.
        results[pct] = df.groupby(['a0', 'b0']).mean(numeric_only=True).reset_index()

    # Fall back to the pre-FOV-sweep single-file layout (no fov_pct column
    # means it predates the FOV sweep -- treat it as the 100% column).
    if not results and os.path.exists('hyperparameter_results.csv'):
        df = pd.read_csv('hyperparameter_results.csv')
        if not df.empty:
            pct = int(round(df['fov_pct'].iloc[0])) if 'fov_pct' in df.columns else 100
            results[pct] = df.groupby(['a0', 'b0']).mean(numeric_only=True).reset_index()

    return dict(sorted(results.items(), key=lambda kv: -kv[0]))  # 100, 75, 50, 25


def pivot_for(df_agg, metric_col):
    pivot = df_agg.pivot(index='a0', columns='b0', values=metric_col)
    # Match the paper's orientation (Supp. Fig. 3/4): a0=0 at the TOP of the
    # y-axis, increasing downward. seaborn.heatmap draws row 0 of the
    # dataframe at the top, so the index must be ascending here.
    return pivot.sort_index(ascending=True)


def draw_heatmap_panel(ax, pivot, vmin=None, vmax=None,
                        show_xticklabels=True, show_yticklabels=True):
    """Draw one square heatmap block: no per-cell gridlines, plain black
    border around the whole block. Returns the mappable for a colorbar.

    Like the paper, tick *numbers* (not axis titles -- those are handled by
    the caller) are only drawn on the outer edge of the panel grid: the
    bottom-left panel is the only one that ends up with both, everything in
    the panel's interior/along the top-right shows neither.
    """
    sns.heatmap(pivot, ax=ax, cmap='afmhot', vmin=vmin, vmax=vmax,
                cbar=False, linewidths=0, linecolor='none')
    ax.set_box_aspect(1)
    for spine in ax.spines.values():
        spine.set_visible(True)
        spine.set_edgecolor(BORDER_COLOR)
        spine.set_linewidth(BORDER_WIDTH)
    if show_xticklabels:
        # Paper style: diagonal beta0 tick labels (there isn't room for them
        # horizontal on a square panel with ~10 unevenly-spaced values).
        ax.set_xticklabels(ax.get_xticklabels(), rotation=45, ha='right')
    ax.tick_params(axis='x', labelbottom=show_xticklabels)
    ax.tick_params(axis='y', labelleft=show_yticklabels)
    return ax.collections[0]


def add_row_colorbar(fig, ax, mappable, symbol, description, ticks):
    """Bordered colorbar with tick numbers on the left (paper style), and
    the metric's symbol + a thin rule + description to its right.

    Sized/positioned manually from ax's actual on-figure bbox rather than
    via mpl_toolkits' make_axes_locatable: box_aspect (used to force the
    heatmap panel square) only resolves at draw time, so querying ax's
    position before a draw gets its pre-square (rectangular) box, and the
    colorbar ends up taller than the panel it belongs to.
    """
    fig.canvas.draw()  # resolve box_aspect so ax.get_position() is the true square box
    pos = ax.get_position()
    # pad needs to fit the tick-number text (now on the left, between the
    # heatmap and the bar) without touching either.
    pad, width = 0.065, 0.018
    cax = fig.add_axes([pos.x1 + pad, pos.y0, width, pos.height])
    cbar = fig.colorbar(mappable, cax=cax, ticks=ticks)

    cax.yaxis.set_ticks_position('left')
    cax.yaxis.set_label_position('left')
    cax.tick_params(labelsize=8)
    # Without this, narrow-range rows (e.g. OverlapRatio, ~0.005-0.03) get a
    # ScalarFormatter offset/multiplier (e.g. "x1e-3") that lands outside
    # this figure's tight custom layout and is easy to lose -- ticks would
    # render as bare "25", "20", ... instead of "0.025", "0.020", ...
    fmt = ScalarFormatter(useOffset=False)
    fmt.set_scientific(False)
    cax.yaxis.set_major_formatter(fmt)
    for spine in cbar.ax.spines.values():
        spine.set_visible(True)
        spine.set_edgecolor(BORDER_COLOR)
        spine.set_linewidth(BORDER_WIDTH)

    label_x = pos.x1 + pad + width + 0.012
    fig.text(label_x, pos.y0 + pos.height * 0.60, symbol,
              ha='left', va='center', fontsize=13)
    fig.text(label_x, pos.y0 + pos.height * 0.50, description,
              ha='left', va='top', fontsize=8.5, linespacing=1.4)
    fig.add_artist(plt.Line2D(
        [label_x, label_x + 0.05], [pos.y0 + pos.height * 0.52] * 2,
        color=BORDER_COLOR, lw=BORDER_WIDTH, transform=fig.transFigure))
    return cax


# ---------------------------------------------------------------------------
# Movement-pattern row (paper Supp. Fig. 3, top row) -- opt-in via --patterns.
# ---------------------------------------------------------------------------
# The paper draws, above the metric heatmaps, a map of the *emergent movement
# pattern* in each (alpha0, beta0) cell. Those labels were assigned BY EYE from
# trajectory plots (the view plot_trajectories.py renders); this sweep keeps no
# trajectories at all (pLogger is off for sweep runs), so all we have per cell
# are the five summary metrics. The classifier below therefore reconstructs the
# labels heuristically from those metrics, and any cell it gets wrong can be
# corrected by hand via patterns_fov<PCT>.csv (see load_pattern_overrides).
#
# Two paper labels are deliberately NOT emitted: SIP (stuck in place) and M
# (milling) both need speed / rotational information that no CSV column
# carries. They stay reachable through the override file.

# key -> (display label, fill colour, description for the legend)
PATTERNS = {
    'X':      ('X',      '#c0392b', 'Unordered'),
    'F':      ('F',      '#6c5ce7', 'Flocking'),
    'frF':    ('frF',    '#b9a7e8', 'fragmented Flocking'),
    'LeFo':   ('LeFo',   '#f6d365', 'Leader-Follower'),
    'frLeFo': ('frLeFo', '#b0b0b0', 'fragmented Leader-Follower'),
    'L':      ('L',      '#f39c12', 'Lines'),
    'S':      ('S',      '#3d3d3d', 'Swarming'),
    'pS':     ('pS',     '#8d8d8d', 'polarized Swarming'),
    # override-only (no metric signature -- see comment above)
    'M':      ('M',      '#2980b9', 'Milling'),
    'SIP':    ('SIP',    '#e8a0a0', 'Stuck In Place'),
}
PATTERN_ORDER = list(PATTERNS)

# All thresholds in one place so they can be retuned without touching the
# decision logic. Calibrated against the FOV-100 sweep and the three anchors
# verified in README.md ("Verified anchors"): (a0,b0)=(0,0) -> X,
# (0.5,0.1) -> F, (0.5,2.0) -> S.
THRESHOLDS = {
    'p_high': 0.50,     # PolarizationOrder: polarized group
    'p_low': 0.32,      #   ... below this, no coherent common direction
    'd_scatter': 0.20,  # MeanDistance / arena_width: group has lost cohesion
    'd_loose': 0.12,    #   ... partially dispersed (subgroups drifting apart)
    'c_cohesive': 0.70, # MaxClusterSize / n_agents: one group
    'c_partial': 0.45,  #   ... a core plus stragglers; below = fragmented
    'rca_line': 0.42,   # AreaToCircleRatio: BELOW this the hull is elongated.
                        # NB uFlockEvaluator's RCA is hull_area/circle_area in
                        # (0,1] -- small = line-like. The paper's RCA of the
                        # same name runs the other way (0..60), so this
                        # threshold is not comparable to the paper's figure.
}


def classify_record(rec, n_agents=10, arena=90.2, thresholds=None):
    """Map one sweep row (the five summary metrics) to a movement-pattern key.

        polarization   cohesion            shape       -> label
        high           cohesive            compact        F
        high           cohesive            elongated      L
        high           partial             -              LeFo
        high           scattered           -              L
        mid            partial/cohesive    -              frF
        mid            fragmented          -              frLeFo
        low            cohesive            -              S
        low            partial             -              pS
        low            fragmented/scattered-              X
    """
    t = dict(THRESHOLDS, **(thresholds or {}))
    p = float(rec['PolarizationOrder'])
    disp = float(rec['MeanDistance']) / float(arena)
    coh = float(rec['MaxClusterSize']) / float(n_agents)
    rca = float(rec['AreaToCircleRatio'])

    # Distance is the cleanest cohesion signal in this data (the X anchor sits
    # at D/arena ~ 0.38 while every cohesive state is below 0.12); the cluster
    # metric only refines "one group" vs "a core plus stragglers". Note
    # uFlockEvaluator's MaxClusterSize runs systematically lower than the
    # paper's (its composite heading+distance linkage rarely puts all ten
    # agents in one cluster), which is why c_partial, not c_cohesive, is what
    # a genuinely tight group has to clear.
    if disp >= t['d_scatter']:
        cohesion = 'scattered'
    elif disp >= t['d_loose']:
        cohesion = 'cohesive' if coh >= t['c_cohesive'] else 'partial'
    elif coh >= t['c_partial']:
        cohesion = 'cohesive'
    else:
        cohesion = 'fragmented'

    if p >= t['p_high']:
        if cohesion == 'cohesive':
            return 'L' if rca < t['rca_line'] else 'F'
        if cohesion == 'partial':
            return 'LeFo'
        # polarized but spread over the torus: the paper's elongated lines
        return 'L'
    if p >= t['p_low']:
        return 'frF' if cohesion in ('cohesive', 'partial') else 'frLeFo'
    if cohesion == 'cohesive':
        return 'S'
    if cohesion == 'partial':
        return 'pS'
    return 'X'


def build_label_grid(records, n_agents=10, arena=90.2):
    """(records) -> (a0 values, b0 values, labels[row][col]).

    Row/column order matches pivot_for(): a0 ascending downward, b0 ascending
    to the right, so the pattern panel lines up cell-for-cell with the metric
    heatmaps below it. Missing combos come back as None (drawn blank).
    """
    a0s = sorted({float(r['a0']) for r in records})
    b0s = sorted({float(r['b0']) for r in records})
    by_cell = {(float(r['a0']), float(r['b0'])): r for r in records}
    labels = [[None] * len(b0s) for _ in a0s]
    for i, a0 in enumerate(a0s):
        for j, b0 in enumerate(b0s):
            rec = by_cell.get((a0, b0))
            if rec is not None:
                labels[i][j] = classify_record(rec, n_agents, arena)
    return a0s, b0s, labels


def load_pattern_overrides(pct):
    """Read hand-assigned labels from patterns_fov<PCT>.csv (a0,b0,label).

    This is the escape hatch for everything the metrics cannot see -- LeFo vs
    frLeFo, milling, stuck-in-place -- filled in by eye from
    plot_trajectories.py, exactly the way the paper's own labels were made.
    """
    path = f"patterns_fov{pct}.csv"
    if not os.path.exists(path):
        return {}
    overrides = {}
    with open(path, newline='') as fh:
        for row in csv.DictReader(fh):
            label = (row.get('label') or '').strip()
            if label not in PATTERNS:
                print(f"   {path}: ignoring unknown label '{label}' "
                      f"(known: {', '.join(PATTERN_ORDER)})")
                continue
            overrides[(float(row['a0']), float(row['b0']))] = label
    print(f"   {path}: {len(overrides)} manual label(s) applied.")
    return overrides


def apply_overrides(a0s, b0s, labels, overrides):
    """Stamp manual labels onto the grid (called before *and* after smoothing,
    so a hand-assigned label is never smoothed away)."""
    if not overrides:
        return labels
    idx_a = {a: i for i, a in enumerate(a0s)}
    idx_b = {b: j for j, b in enumerate(b0s)}
    for (a0, b0), label in overrides.items():
        if a0 in idx_a and b0 in idx_b:
            labels[idx_a[a0]][idx_b[b0]] = label
    return labels


def write_auto_patterns(pct, a0s, b0s, labels):
    """Dump the classifier's labels to patterns_fov<PCT>.auto.csv -- copy to
    patterns_fov<PCT>.csv and hand-correct. Never touches the .csv itself."""
    path = f"patterns_fov{pct}.auto.csv"
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['a0', 'b0', 'label'])
        for i, a0 in enumerate(a0s):
            for j, b0 in enumerate(b0s):
                if labels[i][j] is not None:
                    w.writerow([a0, b0, labels[i][j]])
    print(f"   -> {path} (copy to patterns_fov{pct}.csv to hand-correct)")


def smooth_labels(labels, passes=2):
    """3x3 modal filter, ties resolved in favour of the incumbent label.

    The paper's top row shows contiguous regions, not per-cell noise; single
    stray cells in an otherwise uniform neighbourhood are almost always a
    threshold landing a hair off, not a real pattern island.
    """
    rows, cols = len(labels), len(labels[0])
    for _ in range(passes):
        out = [row[:] for row in labels]
        for i in range(rows):
            for j in range(cols):
                if labels[i][j] is None:
                    continue
                counts = {}
                for di in (-1, 0, 1):
                    for dj in (-1, 0, 1):
                        ii, jj = i + di, j + dj
                        if 0 <= ii < rows and 0 <= jj < cols and labels[ii][jj]:
                            counts[labels[ii][jj]] = counts.get(labels[ii][jj], 0) + 1
                best = max(counts.values())
                if counts.get(labels[i][j], 0) < best:
                    # strict majority against the incumbent -> switch
                    winners = [k for k, v in counts.items() if v == best]
                    if len(winners) == 1:
                        out[i][j] = winners[0]
        labels = out
    return labels


def connected_regions(labels):
    """4-connected same-label components: [(label, cells, (row_c, col_c))]."""
    rows, cols = len(labels), len(labels[0])
    seen = [[False] * cols for _ in range(rows)]
    regions = []
    for i in range(rows):
        for j in range(cols):
            if seen[i][j] or labels[i][j] is None:
                continue
            label, cells, queue = labels[i][j], [], [(i, j)]
            seen[i][j] = True
            while queue:
                ci, cj = queue.pop()
                cells.append((ci, cj))
                for ni, nj in ((ci - 1, cj), (ci + 1, cj), (ci, cj - 1), (ci, cj + 1)):
                    if (0 <= ni < rows and 0 <= nj < cols and not seen[ni][nj]
                            and labels[ni][nj] == label):
                        seen[ni][nj] = True
                        queue.append((ni, nj))
            ci = sum(c[0] for c in cells) / len(cells)
            cj = sum(c[1] for c in cells) / len(cells)
            regions.append((label, cells, (ci, cj)))
    return regions


def _text_colour(hex_colour):
    r, g, b = (int(hex_colour[k:k + 2], 16) / 255 for k in (1, 3, 5))
    return 'black' if (0.299 * r + 0.587 * g + 0.114 * b) > 0.55 else 'white'


def draw_pattern_panel(ax, labels, min_inline_cells=4):
    """Paper-style region map: filled cells, dashed boundaries between
    differing labels, region names at the centroids (small regions get their
    name above the panel with a leader line, as SIP/M/frF do in Supp. Fig. 3).

    `labels` is already smoothed/overridden (see pattern_labels_for).
    Returns the set of pattern keys actually drawn (for the legend).
    """
    rows, cols = len(labels), len(labels[0])
    degenerate = rows < 3 or cols < 3  # no regions to outline on a thin strip

    key_index = {k: i for i, k in enumerate(PATTERN_ORDER)}
    cmap = ListedColormap([PATTERNS[k][1] for k in PATTERN_ORDER])
    grid = [[(key_index[c] if c is not None else float('nan')) for c in row]
            for row in labels]
    ax.imshow(grid, cmap=cmap, vmin=-0.5, vmax=len(PATTERN_ORDER) - 0.5,
              interpolation='nearest', aspect='auto',
              extent=(0, cols, rows, 0))  # same orientation as seaborn.heatmap
    ax.set_xlim(0, cols)
    ax.set_ylim(rows, 0)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_box_aspect(1)
    for spine in ax.spines.values():
        spine.set_visible(True)
        spine.set_edgecolor(BORDER_COLOR)
        spine.set_linewidth(BORDER_WIDTH)

    if not degenerate:
        for i in range(rows):
            for j in range(cols):
                if j + 1 < cols and labels[i][j] != labels[i][j + 1]:
                    ax.plot([j + 1, j + 1], [i, i + 1], color='white',
                            lw=1.2, ls='--', solid_capstyle='butt')
                if i + 1 < rows and labels[i][j] != labels[i + 1][j]:
                    ax.plot([j, j + 1], [i + 1, i + 1], color='white',
                            lw=1.2, ls='--', solid_capstyle='butt')

    used = set()
    tiny = []
    for label, cells, (ci, cj) in connected_regions(labels):
        used.add(label)
        if len(cells) >= min_inline_cells:
            # Put the name on a cell that actually belongs to the region --
            # the centroid of an L-shaped region can fall outside it.
            ri, rj = min(cells, key=lambda c: (c[0] - ci) ** 2 + (c[1] - cj) ** 2)
            ax.text(rj + 0.5, ri + 0.5, PATTERNS[label][0], ha='center', va='center',
                    fontsize=8, fontweight='bold', color=_text_colour(PATTERNS[label][1]))
        else:
            tiny.append((label, cj + 0.5, ci))
    # Small regions: label above the panel with a short leader line down to it.
    tiny.sort(key=lambda t: t[1])
    for n, (label, x, y) in enumerate(tiny):
        y_text = -rows * (0.10 + 0.07 * (n % 2))
        ax.plot([x, x], [y_text * 0.55, y], color=BORDER_COLOR, lw=0.6,
                clip_on=False)
        ax.text(x, y_text, PATTERNS[label][0], ha='center', va='bottom',
                fontsize=6.5, clip_on=False,
                bbox=dict(boxstyle='square,pad=0.15', fc=PATTERNS[label][1],
                          ec=BORDER_COLOR, lw=0.5),
                color=_text_colour(PATTERNS[label][1]))
    return used


def add_pattern_legend(fig, ax, used_keys):
    """"Movement Patterns" key in the same right-hand gutter the metric rows
    put their colorbars in (see add_row_colorbar for why the bbox is read
    after an explicit draw)."""
    fig.canvas.draw()
    pos = ax.get_position()
    x = pos.x1 + 0.065
    y = pos.y1
    fig.text(x, y, 'Movement\nPatterns', ha='left', va='top', fontsize=9.5)
    y -= 0.030
    sw = 0.014
    # Squeeze the entries so the key never runs past the bottom of the pattern
    # panel and into the first metric row's colorbar.
    keys = [k for k in PATTERN_ORDER if k in used_keys]
    gap = min(0.019, max(0.010, (y - pos.y0) / max(len(keys), 1)))
    for key in keys:
        y -= gap
        fig.add_artist(plt.Rectangle((x, y), sw, sw * 0.55, transform=fig.transFigure,
                                     facecolor=PATTERNS[key][1],
                                     edgecolor=BORDER_COLOR, lw=0.5))
        fig.text(x + sw + 0.008, y + sw * 0.27,
                 f"{PATTERNS[key][0]} -- {PATTERNS[key][2]}",
                 ha='left', va='center', fontsize=6.5)


def pattern_labels_for(pct, df_agg, n_agents, arena, dump_auto=True):
    """Per-FOV pipeline: classify every cell, stamp the manual overrides,
    smooth into contiguous regions, then stamp the overrides again so a
    hand-assigned label can never be smoothed away."""
    records = df_agg.to_dict('records')
    a0s, b0s, labels = build_label_grid(records, n_agents, arena)
    overrides = load_pattern_overrides(pct)
    labels = apply_overrides(a0s, b0s, labels, overrides)
    # Too small for a 3x3 neighbourhood vote (e.g. a partial sweep with two a0
    # rows): smoothing a thin strip would invent regions out of nothing.
    if len(a0s) >= 3 and len(b0s) >= 3:
        labels = smooth_labels(labels)
        labels = apply_overrides(a0s, b0s, labels, overrides)
    if dump_auto and not overrides:
        # dump what is actually drawn (post-smoothing), so copying this file to
        # patterns_fov<PCT>.csv reproduces the same figure
        write_auto_patterns(pct, a0s, b0s, labels)
    return labels


def render_single(pct, df_agg, patterns=None):
    """One FOV column. `patterns` is the label grid from pattern_labels_for()
    (or None to reproduce the original, pattern-free figure exactly)."""
    offset = 1 if patterns else 0
    nrows = len(METRICS) + offset
    fig, axes = plt.subplots(nrows=nrows, ncols=1, figsize=(6, 14 + 2.5 * offset))
    # Fix the main grid's position *before* drawing anything into it --
    # add_row_colorbar reads each ax's final on-figure bbox to place its
    # colorbar/label, so the grid can't be allowed to move after that (a
    # later tight_layout() call would shift the heatmap axes but leave the
    # already-placed colorbars/text behind).
    fig.subplots_adjust(left=0.11, right=0.8, top=0.95, bottom=0.05, hspace=0.08)
    plt.suptitle(f'Effects of a0 and b0 on Collective Movement -- FOV {pct}%', fontsize=14, y=0.99)

    if patterns:
        used = draw_pattern_panel(axes[0], patterns)
        add_pattern_legend(fig, axes[0], used)

    last_row = len(METRICS) - 1
    for i, (metric_col, symbol, description, fixed_vmin, fixed_vmax) in enumerate(METRICS):
        ax = axes[i + offset]
        pivot = pivot_for(df_agg, metric_col)
        vmin, vmax, ticks = nice_bounds_and_ticks(
            df_agg[metric_col].min(), df_agg[metric_col].max(), fixed_vmin, fixed_vmax)
        mappable = draw_heatmap_panel(ax, pivot, vmin=vmin, vmax=vmax,
                                       show_xticklabels=(i == last_row), show_yticklabels=True)
        add_row_colorbar(fig, ax, mappable, symbol, description, ticks)
        ax.set_ylabel(r'$\alpha_0$')
        ax.set_xlabel(r'$\beta_0$' if i == last_row else '')

    out = f"01_heatmaps_fov{pct}{'_patterns' if patterns else ''}.png"
    plt.savefig(out, dpi=300, bbox_inches='tight', pad_inches=0.25)
    plt.close(fig)
    print(f"-> {out} erstellt.")


def render_combined(fov_results, patterns=None):
    """All FOVs side by side. `patterns` maps FOV pct -> label grid."""
    ncols = len(fov_results)
    offset = 1 if patterns else 0
    nrows = len(METRICS) + offset
    fig, axes = plt.subplots(nrows=nrows, ncols=ncols,
                             figsize=(3.6 * ncols, 14 + 2.5 * offset))
    if ncols == 1:
        axes = axes.reshape(nrows, 1)
    # See render_single for why this must happen before any drawing.
    # With the pattern row on top, the per-column "FOV x%" titles move up onto
    # it and need room above them for the out-of-panel labels of small regions.
    fig.subplots_adjust(left=0.06, right=0.86, top=0.95 - 0.025 * offset,
                        bottom=0.05, wspace=0.03, hspace=0.09)
    plt.suptitle('Effects of a limited FOV on collective movement (cf. Supp. Fig. 3/4)',
                 fontsize=14, y=0.99 + 0.005 * offset)

    if patterns:
        used = set()
        for col, pct in enumerate(fov_results):
            ax = axes[0][col]
            used |= draw_pattern_panel(ax, patterns[pct])
            # pad clears the out-of-panel labels small regions get
            ax.set_title(f"FOV {pct}%", pad=34)
        add_pattern_legend(fig, axes[0][ncols - 1], used)

    for row, (metric_col, symbol, description, fixed_vmin, fixed_vmax) in enumerate(METRICS):
        # Shared color scale across all FOV columns for this metric, like the
        # paper's single colorbar per row.
        data_min = min(df[metric_col].min() for df in fov_results.values())
        data_max = max(df[metric_col].max() for df in fov_results.values())
        vmin, vmax, ticks = nice_bounds_and_ticks(data_min, data_max, fixed_vmin, fixed_vmax)

        last_row = len(METRICS) - 1
        mappable = None
        for col, (pct, df_agg) in enumerate(fov_results.items()):
            pivot = pivot_for(df_agg, metric_col)
            ax = axes[row + offset][col]
            mappable = draw_heatmap_panel(ax, pivot, vmin=vmin, vmax=vmax,
                                           show_xticklabels=(row == last_row),
                                           show_yticklabels=(col == 0))
            if row == 0 and not patterns:
                ax.set_title(f"FOV {pct}%")
            ax.set_ylabel(r'$\alpha_0$' if col == 0 else '')
            ax.set_xlabel(r'$\beta_0$' if row == last_row else '')

        add_row_colorbar(fig, axes[row + offset][ncols - 1], mappable, symbol, description, ticks)

    out = f"01_figure3_4_heatmaps{'_patterns' if patterns else ''}.png"
    plt.savefig(out, dpi=300, bbox_inches='tight', pad_inches=0.25)
    plt.close(fig)
    print(f"-> {out} erstellt.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--patterns', action='store_true',
                    help="also draw the paper's movement-pattern row on top of the "
                         "heatmaps (labels classified from the summary metrics, "
                         "correctable via patterns_fov<PCT>.csv); writes *_patterns.png")
    ap.add_argument('--agents', type=int, default=10,
                    help='fleet size the sweep was run with (MaxClusterSize scale)')
    ap.add_argument('--arena', type=float, default=90.2,
                    help='arena width in m (MeanDistance scale)')
    args = ap.parse_args()

    fov_results = load_fov_results()
    if not fov_results:
        print("No hyperparameter_results_fov*.csv (or hyperparameter_results.csv) "
              "found -- run optimize.py first.")
        return

    print(f"Found results for FOV(s): {list(fov_results.keys())}")

    patterns = None
    if args.patterns:
        print("Classifying movement patterns from the summary metrics "
              "(M/SIP need trajectories -- set those by hand in patterns_fov<PCT>.csv):")
        patterns = {pct: pattern_labels_for(pct, df, args.agents, args.arena)
                    for pct, df in fov_results.items()}

    for pct, df_agg in fov_results.items():
        render_single(pct, df_agg, patterns[pct] if patterns else None)

    if len(fov_results) > 1:
        render_combined(fov_results, patterns)


if __name__ == "__main__":
    main()
