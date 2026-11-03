#!/usr/bin/env python3
"""Reproducible, unlabelled video regression; candidate counts are not accuracy."""
import argparse
import collections
import csv
import hashlib
import json
import math
import subprocess
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--color', choices=('red', 'blue'), required=True)
    parser.add_argument('--frames', type=int, default=120)
    parser.add_argument('--output', type=Path, required=True, help='new output directory; never overwrite a previous run')
    parser.add_argument('--config', type=Path, default=root / 'src/bof_bringup/config/vision.yaml')
    parser.add_argument('--replay', type=Path, default=root / 'install/bof_vision/lib/bof_vision/bof_replay')
    parser.add_argument('--backend', action='append', choices=('tongji_yolov5', 'tongji_yolov8', 'tongji_yolo11', 'shenzhen_0526', 'shenzhen_0708'))
    args = parser.parse_args()
    if args.frames < 1:
        parser.error('--frames must be positive')
    models = {'tongji_yolov5': root / 'src/models/tongji_yolov5/yolov5.xml',
              'tongji_yolov8': root / 'src/models/tongji_yolov8/yolov8.xml',
              'tongji_yolo11': root / 'src/models/tongji_yolo11/yolo11.xml',
              'shenzhen_0526': root / 'src/models/shenzhen/shenzhen-0526.onnx',
              'shenzhen_0708': root / 'src/models/shenzhen/shenzhen-0708.onnx'}
    selected = list(dict.fromkeys(args.backend or models.keys()))
    video, replay, config = args.video.resolve(strict=True), args.replay.resolve(strict=True), args.config.resolve(strict=True)
    model_files = {backend: [models[backend]] for backend in selected}
    for backend in ('tongji_yolov5', 'tongji_yolov8', 'tongji_yolo11'):
        if backend in selected:
            model_files[backend].append(models[backend].with_suffix('.bin'))
    classifier = root / 'src/models/tongji_yolov8/tiny_resnet.onnx'
    if 'tongji_yolov8' in selected:
        model_files['tongji_yolov8'].append(classifier)
    model_hashes = {backend: {str(path): sha256(path) for path in paths} for backend, paths in model_files.items()}
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest = {'schema_version': 1, 'status': 'running', 'video': str(video), 'video_sha256': sha256(video),
                'replay': str(replay), 'replay_sha256': sha256(replay), 'color': args.color,
                'requested_frames': args.frames, 'mode': 'all', 'device': 'CPU',
                'measurement': 'unlabelled regression; no accuracy/recall claim; timing excludes capture and control',
                'model_sha256': model_hashes, 'results': {}}

    def save():
        (output / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')

    save()
    try:
        for backend in selected:
            csv_path = output / f'{backend}.csv'
            command = [str(replay), '--config', str(config), '--video', str(video), '--color', args.color,
                       '--frames', str(args.frames), '--mode', 'all', '--backend', backend,
                       '--model', str(models[backend]), '--device', 'CPU', '--csv', str(csv_path),
                       '--detections', str(output / f'{backend}.jsonl'), '--preview', str(output / f'{backend}.png')]
            if backend == 'tongji_yolov8':
                command += ['--classifier', str(classifier)]
            manifest['results'][backend] = {'command': command, 'status': 'running'}
            save()
            with (output / f'{backend}.log').open('w') as log:
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
            with csv_path.open() as stream:
                rows = list(csv.DictReader(stream))
            sequences = [int(row['sequence']) for row in rows]
            if not rows or sequences != list(range(len(rows))):
                raise ValueError('expected uninterrupted all-mode sequence from zero')
            times = sorted(float(row['process_ms']) for row in rows[5:])
            percentile = lambda fraction: times[math.ceil(len(times) * fraction) - 1] if times else None
            manifest['results'][backend].update({
                'status': 'complete', 'processed': len(rows),
                'frames_with_candidates': sum(int(row['candidates']) > 0 for row in rows),
                'top_label_counts': dict(collections.Counter(row['top_label'] for row in rows)),
                'timing_samples_after_first_5': len(times), 'process_p50_ms': percentile(0.5),
                'process_p95_ms': percentile(0.95),
                'effective_config': csv_path.name + '.config.yaml'})
            save()
            print(backend, manifest['results'][backend]['top_label_counts'])
        manifest['status'] = 'complete'
    except Exception as error:
        manifest['status'] = 'failed'
        manifest['error'] = str(error)
        raise
    finally:
        save()
    print(f'Regression bundle: {output}')


if __name__ == '__main__':
    main()
