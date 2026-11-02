"""Check metric meaning: classification failure, duplicates, missing frames and corner error."""
import importlib.util
import json
import subprocess
import sys
import tempfile
from pathlib import Path

path = Path(__file__).resolve().parents[4] / 'scripts/dev/evaluate_detections.py'
spec = importlib.util.spec_from_file_location('evaluation', path)
evaluation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluation)


def main():
    corners = [[10, 10], [30, 10], [30, 30], [10, 30]]
    truth = {0: {'sequence': 0, 'width': 100, 'height': 100, 'complete': True,
                 'objects': [{'label': 6, 'corners': corners}]}}
    wrong = {'label': 0, 'corners': corners}
    predictions = {0: {'sequence': 0, 'width': 100, 'height': 100, 'candidates': [wrong, wrong]}}
    result = evaluation.evaluate(predictions, truth)
    assert result['geometry_recall'] == 1 and result['geometry_precision'] == 0.5
    assert result['class_recall'] == 0 and result['class_precision'] == 0
    assert result['unmatched_predictions'] == 1 and result['confusion_true_to_predicted'] == {'6->0': 1}
    shifted = {'label': 6, 'corners': [[x + 1, y] for x, y in corners]}
    predictions[0]['candidates'] = [shifted]
    assert evaluation.evaluate(predictions, truth)['corner_mean_error_px_on_matches'] == 1
    predictions[0]['candidates'] = []
    assert evaluation.evaluate(predictions, truth)['missed_objects'] == 1
    try:
        evaluation.evaluate({}, truth)
        raise AssertionError('missing frames must fail, not improve recall')
    except ValueError:
        pass
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / 'truth.jsonl'
        path.write_text(json.dumps(truth[0]) + '\n')
        assert len(evaluation.read_frames(path, True)) == 1
        predictions[0]['candidates'] = [shifted]
        prediction_path = Path(temporary) / 'predictions.jsonl'
        prediction_path.write_text(json.dumps(predictions[0]) + '\n')
        report = Path(temporary) / 'evaluation.json'
        subprocess.run([sys.executable, str(Path(evaluation.__file__)), '--predictions', str(prediction_path),
                        '--ground-truth', str(path), '--output', str(report)],
                       check=True, capture_output=True, timeout=20)
        assert json.loads(report.read_text())['corner_mean_error_px_on_matches'] == 1
        truth[0]['complete'] = False
        path.write_text(json.dumps(truth[0]) + '\n')
        try:
            evaluation.read_frames(path, True)
            raise AssertionError('partial labels must not count as exhaustive truth')
        except ValueError:
            pass
    print('PASS: evaluation distinguishes geometry, classification, duplicates, misses and corner error')


if __name__ == '__main__':
    main()
