#!/usr/bin/env python3
"""
Generate publication-quality plots for the Results section.
Usage:
    python3 plot_results.py              # defaults to map1
    python3 plot_results.py map2         # plots map2 data
    python3 plot_results.py map3_lte     # plots map3 LTE data
"""
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import os
import sys

matplotlib.rcParams.update({
    'font.family': 'sans-serif',
    'font.sans-serif': ['Arial', 'Helvetica', 'DejaVu Sans'],
    'font.size': 12,
    'axes.titlesize': 14,
    'axes.titleweight': 'bold',
    'axes.labelsize': 12,
    'xtick.labelsize': 11,
    'ytick.labelsize': 12,
    'legend.fontsize': 11,
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.1,
})

RESULTS_DIR = os.path.dirname(os.path.abspath(__file__))
map_id = sys.argv[1] if len(sys.argv) > 1 else 'map1'
suffix = map_id.replace('map', '')

# --- Load data ---
algos = {
    'Baseline':     f'baseline_{suffix}',
    'BGLOSA':       f'bglosa_{suffix}',
    'Green Window': f'glosa_{suffix}',
    'Eco-Driving':  f'eco_{suffix}',
}

colors = {
    'Baseline':     '#6c757d',
    'BGLOSA':       '#0d6efd',
    'Green Window': '#198754',
    'Eco-Driving':  '#dc3545',
}

data = {}
for label, prefix in algos.items():
    for ext in ['_metrics.csv', '.csv']:
        f = os.path.join(RESULTS_DIR, f"{prefix}{ext}")
        if os.path.exists(f):
            data[label] = pd.read_csv(f)
            print(f"  Loaded {label}: {len(data[label])} vehicles from {prefix}{ext}")
            break

if len(data) < 2:
    print(f"ERROR: Need at least 2 conditions loaded, got {len(data)}. "
          f"Did you run simulations for {map_id}?")
    sys.exit(1)

plot_labels = [k for k in algos if k in data]

def apply_modern_style(ax):
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_color('#dddddd')
    ax.spines['bottom'].set_color('#dddddd')
    ax.tick_params(axis='x', length=0)
    ax.yaxis.grid(True, alpha=0.4, linestyle='--')
    ax.set_axisbelow(True)

def make_boxplot(column, ylabel, title, filename):
    fig, ax = plt.subplots(figsize=(8, 5))
    box_data = [data[k][column].values for k in plot_labels]
    
    bp = ax.boxplot(
        box_data, labels=plot_labels, vert=True, patch_artist=True, widths=0.55,
        showmeans=True,
        meanprops=dict(marker='D', markerfacecolor='white',
                       markeredgecolor='black', markersize=6),
        medianprops=dict(color='black', linewidth=1.8),
        whiskerprops=dict(linewidth=1.2),
        capprops=dict(linewidth=1.2),
        flierprops=dict(marker='o', markersize=5, alpha=0.5, markeredgecolor='none', markerfacecolor='black'),
    )
    
    for patch, label in zip(bp['boxes'], plot_labels):
        patch.set_facecolor(colors[label])
        patch.set_alpha(0.85)
        patch.set_edgecolor('black')
        patch.set_linewidth(1.2)
        
    ax.set_ylabel(ylabel, fontweight='semibold')
    ax.set_title(f'{title} ({map_id.upper()})', pad=15)
    apply_modern_style(ax)

    if 'Baseline' in data:
        bl_med = np.median(data['Baseline'][column])
        for i, lab in enumerate(plot_labels):
            if lab == 'Baseline':
                continue
            med = np.median(data[lab][column])
            delta = (med - bl_med) / bl_med * 100
            
            max_val = np.max(data[lab][column])
            ax.annotate(f'{delta:+.1f}%', xy=(i + 1, max_val), xytext=(0, 10),
                        textcoords='offset points', ha='center', fontsize=10,
                        fontweight='bold', color=colors[lab])
            
    plt.tight_layout()
    path = os.path.join(RESULTS_DIR, f'{filename}_{map_id}.png')
    fig.savefig(path)
    print(f"Saved: {os.path.basename(path)}")
    plt.close()

def make_barplot(column, ylabel, title, filename):
    fig, ax = plt.subplots(figsize=(8, 5))
    means = [data[k][column].mean() for k in plot_labels]
    stds = [data[k][column].std() for k in plot_labels]
    x = np.arange(len(plot_labels))
    
    bars = ax.bar(x, means, yerr=stds, width=0.55, capsize=6,
                  color=[colors[l] for l in plot_labels], alpha=0.85,
                  edgecolor='black', linewidth=1.2,
                  error_kw=dict(linewidth=1.5, capthick=1.5, ecolor='#333333'))
    
    ax.set_xticks(x)
    ax.set_xticklabels(plot_labels)
    ax.set_ylabel(ylabel, fontweight='semibold')
    ax.set_title(f'{title} ({map_id.upper()})', pad=15)
    apply_modern_style(ax)

    bl_mean = data['Baseline'][column].mean() if 'Baseline' in data else None
    for i, (lab, m, s) in enumerate(zip(plot_labels, means, stds)):
        ax.text(i, m + s + (max(means)*0.02), f'{m:.1f}', ha='center', va='bottom', fontsize=10, color='#333333')
        
        if lab != 'Baseline' and bl_mean:
            delta = (m - bl_mean) / bl_mean * 100
            ax.text(i, m / 2, f'{delta:+.1f}%', va='center', ha='center', fontsize=11,
                    fontweight='bold', color='white')
            
    plt.tight_layout()
    path = os.path.join(RESULTS_DIR, f'{filename}_{map_id}.png')
    fig.savefig(path)
    print(f"Saved: {os.path.basename(path)}")
    plt.close()


print(f"\nGenerating vertical plots for {map_id.upper()}...\n")

make_boxplot('stopped_time_s', 'Stopped Time (s)',
             'Per-Vehicle Stopped Time at Intersections', 'fig1_stopped_time')

make_boxplot('co2_g', 'CO₂ Emissions (g)',
             'Per-Vehicle CO₂ Emissions', 'fig2_co2')

make_barplot('co2_g', 'Mean CO₂ Emissions (g)',
             'Mean CO₂ Emissions (±1σ)', 'fig3_co2_bar')

make_boxplot('avg_speed_ms', 'Average Speed (m/s)',
             'Per-Vehicle Average Speed', 'fig4_speed')

print(f"\nAll figures for {map_id.upper()} generated successfully!")

