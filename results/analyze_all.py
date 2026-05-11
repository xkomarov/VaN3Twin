"""
Complete analysis of TLM correctness + performance metrics for the Results section.
Usage:
    python3 analyze_all.py              # defaults to map1
    python3 analyze_all.py map2         # analyzes map2 data
    python3 analyze_all.py map3         # analyzes map3 data
    python3 analyze_all.py all          # analyzes all maps + cross-scenario table
"""
import pandas as pd 
import numpy as np
import glob
import os
import sys
from scipy.stats import mannwhitneyu

RESULTS_DIR = os.path.dirname(os.path.abspath(__file__))

def get_algos(map_id):
    suffix = map_id.replace('map', '')
    return {
        'baseline':     f'baseline_{suffix}',
        'bGlosa':       f'bGlosa_{suffix}',
        'greenWindow':  f'greenWindow_{suffix}',
        'ecoDriving':   f'eco_{suffix}',
    }

def analyze_map(map_id):
    ALGOS = get_algos(map_id)
    print(f"\n{'#' * 70}")
    print(f"#  ANALYSIS FOR {map_id.upper()}")
    print(f"{'#' * 70}")

    print("\n" + "=" * 70)
    print(f"SECTION 1: V2X Communication Reliability ({map_id})")
    print("=" * 70)
    for name, prefix in ALGOS.items():
        cum_file = os.path.join(RESULTS_DIR, f"{prefix}_cumulative.csv")
        if os.path.exists(cum_file):
            df = pd.read_csv(cum_file)
            prr = df['avg_PRR'].iloc[0]
            lat = df['avg_latency_ms'].iloc[0]
            print(f"  {name:12s}: PRR = {prr:.4f} ({prr*100:.2f}%),  Latency = {lat:.4f} ms")
        else:
            print(f"  {name:12s}: (no cumulative file)")


    print("\n" + "=" * 70)
    print(f"SECTION 2: TLM Correctness Metrics ({map_id})")
    print("=" * 70)
    for name, prefix in ALGOS.items():
        if name == 'baseline':
            continue
        pattern = os.path.join(RESULTS_DIR, f"{prefix}-*-tlm_correctness.csv")
        files = sorted(glob.glob(pattern))
        if not files:
            print(f"  {name:12s}: No tlm_correctness files found")
            continue
        all_rows = []
        for f in files:
            df = pd.read_csv(f)
            if len(df) > 0:
                all_rows.append(df)
        if not all_rows:
            continue
        combined = pd.concat(all_rows, ignore_index=True)
        total_steps = len(combined)
        phase_match_rate = combined['phaseMatch'].mean() * 100
        phase_mismatch_count = (combined['phaseMatch'] == 0).sum()
        tee_valid = combined[combined['tee_s'] >= 0]['tee_s']
        print(f"\n  === {name.upper()} ===")
        print(f"  Total advisory steps: {total_steps}")
        print(f"  Phase Match Rate:     {phase_match_rate:.2f}%  (mismatches: {phase_mismatch_count})")
        print(f"  TEE: Mean={tee_valid.mean():.3f}s, Median={tee_valid.median():.3f}s, "
              f"Std={tee_valid.std():.3f}s, P95={tee_valid.quantile(0.95):.3f}s")

    print("\n" + "=" * 70)
    print(f"SECTION 3: Vehicle Performance Metrics ({map_id})")
    print("=" * 70)
    perf_data = {}
    for name, prefix in ALGOS.items():
        metrics_file = os.path.join(RESULTS_DIR, f"{prefix}_metrics.csv")
        if not os.path.exists(metrics_file):
            metrics_file = os.path.join(RESULTS_DIR, f"{prefix}.csv")
        if os.path.exists(metrics_file):
            df = pd.read_csv(metrics_file)
            perf_data[name] = df
            co2_g = df['co2_g']
            stopped = df['stopped_time_s']
            avg_speed = df['avg_speed_ms']
            print(f"\n  === {name.upper()} (N={len(df)}) ===")
            print(f"  CO₂ [g]:        mean={co2_g.mean():.2f} ± {co2_g.std():.2f},  median={co2_g.median():.2f}")
            print(f"  Stopped [s]:    mean={stopped.mean():.2f} ± {stopped.std():.2f},  median={stopped.median():.2f}")
            print(f"  Avg speed [m/s]: mean={avg_speed.mean():.2f} ± {avg_speed.std():.2f}")

    if 'baseline' in perf_data:
        print("\n" + "=" * 70)
        print(f"SECTION 4: Percentage Change vs Baseline ({map_id})")
        print("=" * 70)
        bl = perf_data['baseline']
        bl_co2 = bl['co2_g'].mean()
        bl_stopped = bl['stopped_time_s'].mean()
        for name in ['bGlosa', 'greenWindow', 'ecoDriving']:
            if name in perf_data:
                df = perf_data[name]
                delta_co2 = (df['co2_g'].mean() - bl_co2) / bl_co2 * 100
                delta_stopped = (df['stopped_time_s'].mean() - bl_stopped) / bl_stopped * 100
                print(f"  {name.upper():14s} CO₂: {delta_co2:+.2f}%,  Stopped: {delta_stopped:+.2f}%")

    print("\n" + "=" * 70)
    print(f"SECTION 5: Mann-Whitney U Tests ({map_id})")
    print("=" * 70)
    comparisons = [
        ('ecoDriving', 'baseline', 'EcoDriving vs Baseline'),
        ('ecoDriving', 'bGlosa',   'EcoDriving vs BGLOSA'),
        ('ecoDriving', 'greenWindow', 'EcoDriving vs GreenWindow'),
        ('bGlosa', 'baseline', 'BGLOSA vs Baseline'),
        ('greenWindow', 'baseline', 'GreenWindow vs Baseline'),
    ]
    for a, b, label in comparisons:
        if a in perf_data and b in perf_data:
            if len(perf_data[a]) > 0 and len(perf_data[b]) > 0:
                _, p_co2 = mannwhitneyu(perf_data[a]['co2_g'], perf_data[b]['co2_g'], alternative='two-sided')
                _, p_stop = mannwhitneyu(perf_data[a]['stopped_time_s'], perf_data[b]['stopped_time_s'], alternative='two-sided')
                print(f"  {label:20s}  CO₂ p={p_co2:.6f} {'✓' if p_co2<0.05 else '✗'},  "
                      f"Stops p={p_stop:.6f} {'✓' if p_stop<0.05 else '✗'}")
            else:
                print(f"  {label:20s}  Not enough data for Mann-Whitney U test")

    return perf_data


def cross_scenario_table(all_perf):
    """Print cross-scenario summary table."""
    print("\n" + "=" * 70)
    print("CROSS-SCENARIO SUMMARY: Eco-Driving vs Baseline")
    print("=" * 70)
    print(f"  {'Map':<8} {'N':>4} {'Δ CO₂':>10} {'Δ Stops':>10} {'p(CO₂)':>10} {'p(Stops)':>10}")
    print(f"  {'-'*52}")
    for map_id, perf_data in all_perf.items():
        if 'ecoDriving' in perf_data and 'baseline' in perf_data:
            eco = perf_data['ecoDriving']
            bl = perf_data['baseline']
            n = len(eco)
            d_co2 = (eco['co2_g'].mean() - bl['co2_g'].mean()) / bl['co2_g'].mean() * 100 if len(bl) > 0 else float('nan')
            d_stop = (eco['stopped_time_s'].mean() - bl['stopped_time_s'].mean()) / bl['stopped_time_s'].mean() * 100 if len(bl) > 0 else float('nan')
            if len(eco) > 0 and len(bl) > 0:
                _, p_co2 = mannwhitneyu(eco['co2_g'], bl['co2_g'], alternative='two-sided')
                _, p_stop = mannwhitneyu(eco['stopped_time_s'], bl['stopped_time_s'], alternative='two-sided')
            else:
                p_co2, p_stop = float('nan'), float('nan')
            print(f"  {map_id:<8} {n:>4} {d_co2:>+9.2f}% {d_stop:>+9.2f}% {p_co2:>10.6f} {p_stop:>10.6f}")


class Tee(object):
    def __init__(self, *files):
        self.files = files
    def write(self, obj):
        for f in self.files:
            f.write(obj)
    def flush(self):
        for f in self.files:
            f.flush()

if __name__ == '__main__':
    arg = sys.argv[1] if len(sys.argv) > 1 else 'map1'
    

    report_name = f"analysis_{arg}.txt"
    report_path = os.path.join(RESULTS_DIR, report_name)
    
    with open(report_path, 'w', encoding='utf-8') as f:
        original_stdout = sys.stdout
        sys.stdout = Tee(sys.stdout, f)
        
        try:
            if arg == 'all':
                all_perf = {}
                for m in ['map1', 'map2', 'map3']:
                    all_perf[m] = analyze_map(m)
                cross_scenario_table(all_perf)
            elif arg == 'all_lte':
                all_perf = {}
                for m in ['map1_lte', 'map2_lte', 'map3_lte']:
                    all_perf[m] = analyze_map(m)
                cross_scenario_table(all_perf)
            else:
                analyze_map(arg)

            print("\n" + "=" * 70)
            print("ANALYSIS COMPLETE")
            print(f"Results saved to: {report_path}")
            print("=" * 70)
        finally:
            sys.stdout = original_stdout
