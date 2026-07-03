import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

print("Lese hyperparameter_results.csv ein...")
df = pd.read_csv('hyperparameter_results.csv')

# --- PLOT: Figure 4 Style Heatmaps ---
# The paper plots a0 on the y-axis and b0 on the x-axis. 
# Since a1 and b1 also vary in your dataset, we will aggregate them by taking the mean.
df_agg = df.groupby(['a0', 'b0']).mean().reset_index()

metrics = [
    ('PolarizationOrder', 'P\nPolarization\nOrder'),
    ('MeanDistance', 'D\nMean\nInter-individual\nDistance'),
    ('MaxClusterSize', 'N_clus^{max}\nSize of\nLargest cluster'),
    ('AreaToCircleRatio', 'RCA\nArea-to-Circle\nRatio'),
    ('OverlapRatio', 'R_o^{sim}\nTime Ratio\nIn Overlap')
]

# Set up the figure grid (5 rows, 1 column)
# If you add FOV later, you can increase the number of columns.
fig, axes = plt.subplots(nrows=len(metrics), ncols=1, figsize=(6, 12), sharex=True)

for i, (metric_col, metric_label) in enumerate(metrics):
    # Pivot the data for the heatmap
    pivot_table = df_agg.pivot(index='a0', columns='b0', values=metric_col)
    
    # Sort index descending so the lowest a0 value is at the bottom of the y-axis
    pivot_table = pivot_table.sort_index(ascending=False)
    
    # The paper uses a black-red-yellow-white colormap, 'afmhot' is the closest matplotlib equivalent
    sns.heatmap(
        pivot_table, 
        ax=axes[i], 
        cmap='afmhot', 
        cbar=True, 
        cbar_kws={"shrink": 0.8}
    )
    
    # Formatting to match the paper's style
    axes[i].set_ylabel(r'$\alpha_0$')
    if i == len(metrics) - 1:
        axes[i].set_xlabel(r'$\beta_0$')
    else:
        axes[i].set_xlabel('')
        
    # Place the metric name on the right side of the heatmap
    axes[i].text(1.15, 0.5, metric_label, transform=axes[i].transAxes, 
                 va='center', ha='left', fontsize=10)

plt.suptitle('Effects of a0 and b0 on Collective Movement', fontsize=14, y=0.98)
plt.tight_layout(rect=[0, 0, 0.85, 0.95]) # Leave space on the right for the text labels
plt.savefig("01_figure4_heatmaps.png", dpi=300, bbox_inches='tight')
plt.close()
print("-> 01_figure4_heatmaps.png erstellt.")