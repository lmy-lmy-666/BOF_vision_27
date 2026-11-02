"""Exercise the real executable: override order, invalid input and reproducibility."""
import csv
import json
import subprocess
import sys
import tempfile
from pathlib import Path


def main():
    executable = str(Path(sys.argv[1]).resolve())
    with tempfile.TemporaryDirectory(prefix='bof-config-cli-') as temporary:
        root = Path(temporary)
        config = root / 'blue.yaml'
        config.write_text('enemy_color: blue\ntrack_timeout_s: 0.0001\ntrack_confirm_frames: 1\nassociation_gate_px: 50.0\n')

        def run(*args, succeeds=True):
            result = subprocess.run([executable, *map(str, args)], cwd=root,
                                    text=True, capture_output=True, timeout=20)
            assert (result.returncode == 0) == succeeds, result.stdout + result.stderr
            return result

        before, after = root / 'before.yaml', root / 'after.yaml'
        run('--color', 'red', '--config', config, '--dump-config', before, '--check-config')
        run('--config', config, '--color', 'red', '--dump-config', after, '--check-config')
        assert before.read_text() == after.read_text()
        assert 'enemy_color: red' in before.read_text()
        output = root / 'frames.csv'
        detections = root / 'detections.jsonl'
        run('--config', config, '--frames', '12', '--csv', output, '--detections', detections)
        with output.open() as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 12
        assert all(row['valid'] == '1' and row['candidates'] == '1' for row in rows)
        # A timeout shorter than the nominal frame interval must reset each association.
        assert len({row['track_id'] for row in rows}) == 12
        frames = [json.loads(line) for line in detections.read_text().splitlines()]
        assert [frame['sequence'] for frame in frames] == list(range(12))
        assert all(len(frame['candidates']) == 1 and len(frame['candidates'][0]['corners']) == 4 for frame in frames)
        run('--frames', '2', '--color-threshold', '255', '--detections', detections)
        assert all(not json.loads(line)['candidates'] for line in detections.read_text().splitlines())
        snapshot = Path(str(output) + '.config.yaml')
        assert 'enemy_color: blue' in snapshot.read_text()
        run('--config', snapshot, '--check-config')
        for args in [('--frames', '12oops'), ('--fps', 'nan'), ('--track-timeout-s', '-1'),
                     ('--confidence', '1.1'), ('--confidence', '0'), ('--nms', '1'),
                     ('--nms', '-0.1'), ('--color-threshold', '256'),
                     ('--config', root / 'missing.yaml'), ('--unknown', '1')]:
            run(*args, succeeds=False)
        config.write_text('confidence_thresold: 0.5\n')
        result = run('--config', config, '--camera', 'auto', succeeds=False)
        assert 'unknown configuration field' in result.stderr
        print('PASS: CLI configuration, snapshots, effective tracker options and failure paths')


if __name__ == '__main__':
    main()
