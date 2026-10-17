"""Summarise benchmark.csv into a markdown report + charts.

    ros2 run pai_benchmark report --csv ~/pai_results/benchmark.csv --out ~/pai_results
"""
import argparse
import csv
import os
import statistics
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

SYSTEMS = [('claude', 'AI pipeline (Claude planner)'), ('rule', 'AI pipeline (offline rule planner)'),
           ('baseline', 'Traditional taught cell')]
CONDITIONS = ['nominal', 'pose', 'lighting', 'clutter', 'disturbance', 'novel']
# Reference categorical palette, slots 1-3 in fixed order (documented as passing the CVD checks).
COLORS = {'claude': '#2a78d6', 'rule': '#eb6834', 'baseline': '#1baf7a'}
INK, MUTED, GRID, SURFACE = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'


def load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def pct(xs):
    return 100.0 * sum(xs) / len(xs) if xs else float('nan')


def med(xs):
    xs = [float(x) for x in xs if x not in ('', None)]
    return statistics.median(xs) if xs else float('nan')


def style(ax):
    ax.set_facecolor(SURFACE)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.yaxis.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default=os.path.expanduser('~/pai_results/benchmark.csv'))
    ap.add_argument('--out', default=os.path.expanduser('~/pai_results'))
    args, _ = ap.parse_known_args()
    rows = load(args.csv)
    systems = [(k, n) for k, n in SYSTEMS if any(r['system'] == k for r in rows)]
    conds = [c for c in CONDITIONS if any(r['condition'] == c for r in rows)]
    by = defaultdict(list)
    for r in rows:
        by[(r['system'], r['condition'])].append(r)

    # ---- success-rate chart
    fig, ax = plt.subplots(figsize=(9, 3.8), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    style(ax)
    width = 0.8 / len(systems)
    for i, (key, name) in enumerate(systems):
        xs = [j + (i - (len(systems) - 1) / 2) * width for j in range(len(conds))]
        ys = [pct([int(r['success']) for r in by[(key, c)]]) for c in conds]
        ax.bar(xs, ys, width=width - 0.02, color=COLORS[key], label=name, edgecolor=SURFACE, linewidth=1)
        for x, y in zip(xs, ys):
            if y == 0:          # a 0 % bar is invisible: say so explicitly
                ax.text(x, 1.5, '0%', ha='center', va='bottom', color=MUTED, fontsize=8)
    ax.set_xticks(range(len(conds)))
    ax.set_xticklabels(conds, color=INK, fontsize=10)
    ax.set_ylim(0, 105)
    ax.set_ylabel('Task success (%)', color=MUTED, fontsize=9)
    ax.set_title('Task success by test condition (scored on simulator ground truth)', color=INK, fontsize=11,
                 loc='left', pad=12)
    ax.legend(frameon=False, fontsize=9, loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=len(systems),
              labelcolor=INK)
    fig.tight_layout()
    chart1 = os.path.join(args.out, 'success_by_condition.png')
    fig.savefig(chart1, facecolor=SURFACE)
    plt.close(fig)

    # ---- timing chart (successful trials only): median time to first motion and total time
    fig, axes = plt.subplots(1, 2, figsize=(9, 2.6), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    for ax, field, title in ((axes[0], 'time_to_first_motion', 'Median time to first motion (s)'),
                             (axes[1], 'total_time', 'Median total task time (s)')):
        style(ax)
        ax.yaxis.grid(False)
        ax.xaxis.grid(True, color=GRID, linewidth=0.8)
        vals = [med([r[field] for r in rows if r['system'] == k and r['success'] == '1']) for k, _ in systems]
        ax.barh(range(len(systems)), vals, color=[COLORS[k] for k, _ in systems], height=0.6)
        ax.set_yticks(range(len(systems)))
        ax.set_yticklabels([n for _, n in systems], color=INK, fontsize=8)
        ax.invert_yaxis()
        ax.set_title(title, color=INK, fontsize=10, loc='left')
        for y, v in enumerate(vals):
            if v == v:
                ax.text(v, y, f' {v:.1f}', va='center', color=INK, fontsize=8)
    fig.tight_layout()
    chart2 = os.path.join(args.out, 'timing.png')
    fig.savefig(chart2, facecolor=SURFACE)
    plt.close(fig)

    # ---- markdown
    lines = ['# Benchmark results', '',
             f'{len(rows)} trials. A trial succeeds only if every goal holds in Gazebo afterwards and no other part '
             'was moved by more than 3 cm.', '', '![success](success_by_condition.png)', '',
             '## Success rate by condition', '',
             '| condition | ' + ' | '.join(n for _, n in systems) + ' |',
             '|---|' + '---|' * len(systems)]
    for c in conds:
        cells = []
        for k, _ in systems:
            rs = by[(k, c)]
            cells.append(f'{pct([int(r["success"]) for r in rs]):.0f}% ({sum(int(r["success"]) for r in rs)}/{len(rs)})'
                         if rs else '-')
        lines.append(f'| {c} | ' + ' | '.join(cells) + ' |')
    overall = []
    for k, _ in systems:
        rs = [r for r in rows if r['system'] == k]
        overall.append(f'**{pct([int(r["success"]) for r in rs]):.0f}%** ({sum(int(r["success"]) for r in rs)}/{len(rs)})')
    lines.append('| **all** | ' + ' | '.join(overall) + ' |')
    lines += ['', '## Other metrics (all trials)', '',
              '| metric | ' + ' | '.join(n for _, n in systems) + ' |', '|---|' + '---|' * len(systems)]

    def metric(label, fn):
        lines.append(f'| {label} | ' + ' | '.join(fn([r for r in rows if r['system'] == k]) for k, _ in systems) + ' |')
    metric('goals achieved', lambda rs: f'{100 * sum(int(r["goals_met"]) for r in rs) / max(1, sum(int(r["goals"]) for r in rs)):.0f}%')
    metric('reported success but wrong end state', lambda rs: str(sum(1 for r in rs if r['reported_success'] == '1' and r['success'] == '0')))
    metric('other parts disturbed', lambda rs: str(sum(1 for r in rs if r['collateral_moves'])))
    metric('asked for clarification', lambda rs: str(sum(int(r['clarified']) for r in rs)))
    metric('median planning time (s)', lambda rs: f'{med([r["planning_time"] for r in rs]):.2f}')
    metric('median time to first motion (s)', lambda rs: f'{med([r["time_to_first_motion"] for r in rs if r["success"] == "1"]):.1f}')
    metric('median total time, successful (s)', lambda rs: f'{med([r["total_time"] for r in rs if r["success"] == "1"]):.0f}')
    metric('trials that re-planned', lambda rs: str(sum(1 for r in rs if r['replans'] not in ('', '0'))))
    metric('safety stops', lambda rs: str(sum(int(r['safety_stops'] or 0) for r in rs)))
    lines += ['', '![timing](timing.png)', '', '## Failures', '']
    for r in rows:
        if r['success'] == '0':
            lines.append(f'- `{r["system"]}` / {r["condition"]} / {r["task"]} #{r["trial"]}: {r["summary"]}')
    md = os.path.join(args.out, 'RESULTS.md')
    with open(md, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'wrote {md}, {chart1}, {chart2}')
