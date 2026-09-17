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
import glob
import math
import os
import re

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
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


def render_single(pct, df_agg):
    fig, axes = plt.subplots(nrows=len(METRICS), ncols=1, figsize=(6, 14))
    # Fix the main grid's position *before* drawing anything into it --
    # add_row_colorbar reads each ax's final on-figure bbox to place its
    # colorbar/label, so the grid can't be allowed to move after that (a
    # later tight_layout() call would shift the heatmap axes but leave the
    # already-placed colorbars/text behind).
    fig.subplots_adjust(left=0.11, right=0.8, top=0.95, bottom=0.05, hspace=0.08)
    plt.suptitle(f'Effects of a0 and b0 on Collective Movement -- FOV {pct}%', fontsize=14, y=0.99)

    last_row = len(METRICS) - 1
    for i, (metric_col, symbol, description, fixed_vmin, fixed_vmax) in enumerate(METRICS):
        pivot = pivot_for(df_agg, metric_col)
        vmin, vmax, ticks = nice_bounds_and_ticks(
            df_agg[metric_col].min(), df_agg[metric_col].max(), fixed_vmin, fixed_vmax)
        mappable = draw_heatmap_panel(axes[i], pivot, vmin=vmin, vmax=vmax,
                                       show_xticklabels=(i == last_row), show_yticklabels=True)
        add_row_colorbar(fig, axes[i], mappable, symbol, description, ticks)
        axes[i].set_ylabel(r'$\alpha_0$')
        axes[i].set_xlabel(r'$\beta_0$' if i == last_row else '')

    out = f"01_heatmaps_fov{pct}.png"
    plt.savefig(out, dpi=300, bbox_inches='tight', pad_inches=0.25)
    plt.close(fig)
    print(f"-> {out} erstellt.")


def render_combined(fov_results):
    ncols = len(fov_results)
    fig, axes = plt.subplots(nrows=len(METRICS), ncols=ncols, figsize=(3.6 * ncols, 14))
    if ncols == 1:
        axes = axes.reshape(len(METRICS), 1)
    # See render_single for why this must happen before any drawing.
    fig.subplots_adjust(left=0.06, right=0.86, top=0.95, bottom=0.05, wspace=0.03, hspace=0.09)
    plt.suptitle('Effects of a limited FOV on collective movement (cf. Supp. Fig. 3/4)', fontsize=14, y=0.99)

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
            ax = axes[row][col]
            mappable = draw_heatmap_panel(ax, pivot, vmin=vmin, vmax=vmax,
                                           show_xticklabels=(row == last_row),
                                           show_yticklabels=(col == 0))
            if row == 0:
                ax.set_title(f"FOV {pct}%")
            ax.set_ylabel(r'$\alpha_0$' if col == 0 else '')
            ax.set_xlabel(r'$\beta_0$' if row == last_row else '')

        add_row_colorbar(fig, axes[row][ncols - 1], mappable, symbol, description, ticks)

    out = "01_figure3_4_heatmaps.png"
    plt.savefig(out, dpi=300, bbox_inches='tight', pad_inches=0.25)
    plt.close(fig)
    print(f"-> {out} erstellt.")


def main():
    fov_results = load_fov_results()
    if not fov_results:
        print("No hyperparameter_results_fov*.csv (or hyperparameter_results.csv) "
              "found -- run optimize.py first.")
        return

    print(f"Found results for FOV(s): {list(fov_results.keys())}")
    for pct, df_agg in fov_results.items():
        render_single(pct, df_agg)

    if len(fov_results) > 1:
        render_combined(fov_results)


if __name__ == "__main__":
    main()
