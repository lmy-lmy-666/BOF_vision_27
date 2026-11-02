#!/usr/bin/env python3
"""Summarize a bof_live field session; never replace actual calibration with fixtures."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path)
    parser.add_argument('--reference-distance-m', type=float, help='Measured optical-centre to plate-centre slant distance, not axial z')
    parser.add_argument('--label', type=int, help='Only one isolated target of this label may be present')
    parser.add_argument('--max-relative-error', type=float, default=0.05)
    parser.add_argument('--min-samples', type=int, default=30)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if (not math.isfinite(args.max_relative_error) or not 0 < args.max_relative_error < 1
            or args.min_samples < 1 or (args.reference_distance_m is not None
            and (not math.isfinite(args.reference_distance_m) or args.reference_distance_m <= 0))):
        parser.error('invalid distance/sample/tolerance bounds')
    report = {'status': 'PENDING', 'scope': 'single stationary-distance vision check; not complete aiming acceptance',
              'pending': [], 'files_sha256': {}, 'distance': {}, 'capture': {}}
    if not args.session:
        report['pending'].append('No real camera session provided')
    else:
        session = args.session.resolve()
        report['session'] = str(session)
        for name in ('session.yaml', 'effective_config.yaml', 'calibration_used.yaml', 'observations.csv', 'poses.csv'):
            path = session / name
            if path.is_file():
                report['files_sha256'][name] = hashlib.sha256(path.read_bytes()).hexdigest()
            else:
                report['pending'].append('Missing ' + name)
        meta_path = session / 'session.yaml'
        meta = yaml.safe_load(meta_path.read_text()) if meta_path.is_file() else {}
        report['source'] = meta.get('source', 'unknown')
        if report['source'] not in ('hik', 'usb'):
            report['pending'].append('Session is not a direct hardware capture')
        metrics_path = session / 'observations.csv'
        if metrics_path.is_file():
            with metrics_path.open() as stream:
                metrics = list(csv.DictReader(stream))
            def finite_values(key):
                values = [float(row[key]) for row in metrics]
                if not all(math.isfinite(v) for v in values):
                    raise ValueError('Nonfinite metrics: ' + key)
                return values
            if metrics:
                stamps = finite_values('stamp_ns')
                delta = [(b-a)*1e-9 for a,b in zip(stamps, stamps[1:])]
                report['capture'] = {'processed_frames': len(metrics),
                    'monotonic_timestamps': all(v > 0 for v in delta),
                    'mean_processing_ms': statistics.mean(finite_values('process_ms')),
                    'mean_receive_to_result_ms': statistics.mean(finite_values('receive_to_result_ms')),
                    'generations': sorted({row['capture_generation'] for row in metrics})}
                if delta and all(v > 0 for v in delta):
                    report['capture']['processed_fps'] = 1 / statistics.mean(delta)
                if not all(v > 0 for v in delta):
                    report['pending'].append('Nonmonotonic capture timestamps')
            else:
                report['pending'].append('No processed frames')
        if args.reference_distance_m is None or args.label is None:
            report['pending'].append('Measured slant distance and target label are required')
        pose_path = session / 'poses.csv'
        if pose_path.is_file() and args.reference_distance_m is not None and args.label is not None:
            with pose_path.open() as stream:
                rows = [r for r in csv.DictReader(stream) if int(r['label']) == args.label]
            values, seen = [], set()
            ambiguous = 0
            for row in rows:
                if row['valid'] not in ('1', 'true'):
                    continue
                key = (row['capture_generation'], row['sequence'])
                if key in seen:
                    raise ValueError('Multiple same-label targets in a frame; capture one isolated target')
                seen.add(key)
                ambiguous += row['ambiguous'] in ('1', 'true')
                value = float(row['distance_m'])
                if not math.isfinite(value) or value <= 0:
                    raise ValueError('Invalid distance in poses.csv')
                values.append(value)
            errors = sorted(abs(v - args.reference_distance_m) / args.reference_distance_m for v in values)
            report['distance'] = {'samples': len(values), 'label': args.label,
                'reference_slant_distance_m': args.reference_distance_m,
                'max_relative_error': args.max_relative_error, 'ambiguous_orientation_samples': ambiguous}
            if len(values) < args.min_samples:
                report['pending'].append('Insufficient valid distance samples')
            if values:
                p95 = errors[math.ceil(len(errors)*0.95)-1]
                report['distance'].update(median_m=statistics.median(values), p95_relative_error=p95,
                                          within_tolerance=p95 <= args.max_relative_error)
                if p95 > args.max_relative_error:
                    report['status'] = 'FAIL'
        if not report['pending'] and report['status'] != 'FAIL':
            report['status'] = 'PASS_SINGLE_DISTANCE'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Keep earlier evidence; choose a new report name for subsequent captures.
    with args.output.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['status'] == 'PASS_SINGLE_DISTANCE' else 1 if report['status'] == 'FAIL' else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, yaml.YAMLError) as error:
        print('field report failed: ' + str(error), file=sys.stderr)
        sys.exit(1)
