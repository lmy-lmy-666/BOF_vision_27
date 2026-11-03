#!/usr/bin/env python3
"""Evaluate exhaustive labelled frames using class-independent greedy polygon-IoU matching."""
import argparse
import collections
import json
import math
from pathlib import Path

import cv2
import numpy as np


def read_frames(path, ground_truth=False):
    frames = {}
    key = 'objects' if ground_truth else 'candidates'
    with Path(path).open() as stream:
        for line_number, line in enumerate(stream, 1):
            frame = json.loads(line)
            sequence = frame['sequence']
            if type(sequence) is not int or sequence < 0 or sequence in frames:
                raise ValueError(f'{path}:{line_number}: invalid or duplicate sequence')
            if any(type(frame[name]) is not int or frame[name] <= 0 for name in ('width', 'height')):
                raise ValueError('width and height must be positive integers')
            if ground_truth and frame.get('complete') is not True:
                raise ValueError('each labelled frame must declare complete: true (exhaustive enemy annotations)')
            for item in frame[key]:
                if type(item['label']) is not int or item['label'] not in range(0 if ground_truth else -1, 8):
                    raise ValueError('invalid semantic label')
                corners = np.asarray(item['corners'], dtype=np.float32)
                if corners.shape != (4, 2) or not np.isfinite(corners).all():
                    raise ValueError('expected four finite TL/TR/BR/BL corners')
                if not cv2.isContourConvex(corners) or abs(cv2.contourArea(corners)) <= 0:
                    raise ValueError('corners must form a non-degenerate convex quadrilateral')
            frames[sequence] = frame
    if not frames:
        raise ValueError(f'no frames: {path}')
    return frames


def evaluate(predictions, truth, threshold=0.5):
    if not math.isfinite(threshold) or not 0 < threshold <= 1:
        raise ValueError('IoU threshold must be in (0,1]')
    missing = set(truth) - set(predictions)
    if missing:
        raise ValueError(f'predictions missing labelled frames: {sorted(missing)[:10]}')
    matched = correct = predicted = labelled = 0
    corner_errors, confusion = [], collections.Counter()
    for sequence, target in truth.items():
        prediction = predictions[sequence]
        if (target['width'], target['height']) != (prediction['width'], prediction['height']):
            raise ValueError(f'frame {sequence}: resolution mismatch')
        actual, expected = prediction['candidates'], target['objects']
        predicted += len(actual); labelled += len(expected)
        pairs = []
        for i, candidate in enumerate(actual):
            a = np.asarray(candidate['corners'], dtype=np.float32)
            for j, obj in enumerate(expected):
                b = np.asarray(obj['corners'], dtype=np.float32)
                intersection, _ = cv2.intersectConvexConvex(a, b)
                union = abs(cv2.contourArea(a)) + abs(cv2.contourArea(b)) - intersection
                iou = intersection / union if union > 0 else 0
                if iou >= threshold:
                    pairs.append((-iou, i, j))
        used_actual, used_expected = set(), set()
        for _, i, j in sorted(pairs):
            if i in used_actual or j in used_expected:
                continue
            used_actual.add(i); used_expected.add(j); matched += 1
            is_correct = actual[i]['label'] == expected[j]['label']
            correct += is_correct
            confusion[f"{expected[j]['label']}->{actual[i]['label']}"] += 1
            delta = np.asarray(actual[i]['corners']) - np.asarray(expected[j]['corners'])
            corner_errors.extend(np.linalg.norm(delta, axis=1).tolist())
    ratio = lambda numerator, denominator: numerator / denominator if denominator else None
    return {'schema_version': 1, 'matching': 'class-independent greedy descending polygon IoU; not COCO mAP',
            'iou_threshold': threshold, 'labelled_frames': len(truth), 'prediction_frames': len(predictions),
            'unlabelled_frames_excluded': len(set(predictions) - set(truth)),
            'ground_truth_objects': labelled, 'predicted_objects_on_labelled_frames': predicted,
            'geometry_matches': matched, 'missed_objects': labelled - matched, 'unmatched_predictions': predicted - matched,
            'geometry_precision': ratio(matched, predicted), 'geometry_recall': ratio(matched, labelled),
            'correct_class_matches': correct, 'class_precision': ratio(correct, predicted),
            'class_recall': ratio(correct, labelled), 'classification_accuracy_on_matches': ratio(correct, matched),
            'confusion_true_to_predicted': dict(sorted(confusion.items())),
            'corner_mean_error_px_on_matches': ratio(sum(corner_errors), len(corner_errors)),
            'corner_p95_error_px_on_matches': sorted(corner_errors)[math.ceil(len(corner_errors) * 0.95) - 1] if corner_errors else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--predictions', required=True, type=Path)
    parser.add_argument('--ground-truth', required=True, type=Path)
    parser.add_argument('--iou', type=float, default=0.5)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = evaluate(read_frames(args.predictions), read_frames(args.ground_truth, True), args.iou)
    result['predictions'] = str(args.predictions.resolve())
    result['ground_truth'] = str(args.ground_truth.resolve())
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
