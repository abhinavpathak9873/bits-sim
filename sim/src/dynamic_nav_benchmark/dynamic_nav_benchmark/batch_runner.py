import argparse
import csv
import itertools
import json
import time
from pathlib import Path
from typing import Sequence

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from dynamic_nav_benchmark.experiment_runner import load_config, run_dry_trial, run_ros_trial


def _values(config: dict, key: str, default):
    batch = config.get('batch', {})
    return batch.get(key, [config.get(key, default)])


def _variant_config(base: dict, seed, density, world, goal_index: int) -> dict:
    config = dict(base)
    config['seed'] = seed
    config['crowd_density'] = density
    config['world'] = world
    if goal_index is not None:
        config['goals'] = [base['goals'][goal_index]]
    return config


def write_batch_outputs(summaries: list[dict], output_dir: Path, batch_id: str) -> None:
    batch_dir = output_dir / batch_id
    batch_dir.mkdir(parents=True, exist_ok=True)
    with (batch_dir / 'batch_summary.json').open('w', encoding='utf-8') as stream:
        json.dump(summaries, stream, indent=2)
    with (batch_dir / 'batch_summary.csv').open('w', newline='', encoding='utf-8') as stream:
        fields = [
            'run_id', 'world', 'backend', 'robot', 'seed', 'crowd_density', 'success',
            'duration_s', 'path_length_m', 'collision_count', 'near_miss_count',
            'minimum_obstacle_distance_m', 'final_pose_error_m',
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for summary in summaries:
            writer.writerow({field: summary.get(field) for field in fields})
    if summaries:
        labels = [summary['run_id'] for summary in summaries]
        durations = [summary.get('duration_s') or 0.0 for summary in summaries]
        path_lengths = [summary.get('path_length_m') or 0.0 for summary in summaries]
        plt.figure(figsize=(max(8, len(labels) * 0.7), 4))
        plt.bar(labels, durations)
        plt.xticks(rotation=45, ha='right')
        plt.ylabel('duration [s]')
        plt.tight_layout()
        plt.savefig(batch_dir / 'duration_sweep.png', dpi=140)
        plt.close()
        plt.figure(figsize=(max(8, len(labels) * 0.7), 4))
        plt.bar(labels, path_lengths)
        plt.xticks(rotation=45, ha='right')
        plt.ylabel('path length [m]')
        plt.tight_layout()
        plt.savefig(batch_dir / 'path_length_sweep.png', dpi=140)
        plt.close()


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--output-dir', default='results')
    parser.add_argument('--batch-id')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)

    base = load_config(Path(args.config))
    batch_id = args.batch_id or base.get('batch_id') or f"batch_{int(time.time())}"
    output_dir = Path(args.output_dir)
    seeds = _values(base, 'seeds', base.get('seed', 1))
    densities = _values(base, 'crowd_densities', base.get('crowd_density', 'medium'))
    worlds = _values(base, 'worlds', base.get('world', 'mall'))
    goal_indices = base.get('batch', {}).get('goal_indices', [0])

    summaries = []
    for seed, density, world, goal_index in itertools.product(seeds, densities, worlds, goal_indices):
        config = _variant_config(base, seed, density, world, int(goal_index))
        run_id = f"{batch_id}_{world}_{density}_seed{seed}_goal{goal_index}"
        if args.dry_run:
            summary = run_dry_trial(config, run_id, output_dir)
        else:
            summary = run_ros_trial(config, run_id, output_dir)
        summaries.append(summary)
    write_batch_outputs(summaries, output_dir, batch_id)
    print(json.dumps({'batch_id': batch_id, 'runs': len(summaries)}, indent=2))


if __name__ == '__main__':
    main()
