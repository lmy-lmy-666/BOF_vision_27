"""Black-box ROS integration check. Not part of the deployed C++ pipeline."""
import argparse
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from bof_interfaces.msg import ArmorCandidates, PixelTrack
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--video', default='')
    parser.add_argument('--backend', default='light_pair')
    parser.add_argument('--model', default='')
    parser.add_argument('--color', choices=('red', 'blue'), default='red')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('bof_smoke_observer')
    tracks, candidates, images = [], [], []
    subscriptions = [
        node.create_subscription(PixelTrack, '/perception/pixel_track', tracks.append, qos_profile_sensor_data),
        node.create_subscription(ArmorCandidates, '/perception/candidates', candidates.append, qos_profile_sensor_data),
        node.create_subscription(Image, '/debug/image', images.append, qos_profile_sensor_data),
    ]
    processes = []

    def spin_until(predicate, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError('timed out waiting for ROS output')

    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()

    with tempfile.TemporaryDirectory(prefix='bof-smoke-') as directory:
        logs = open(Path(directory) / 'nodes.log', 'w+')
        try:
            pipeline_command = ['ros2', 'run', 'bof_vision', 'pipeline_node', '--ros-args',
                                '-p', f'detector_backend:={args.backend}', '-p', f'enemy_color:={args.color}']
            if args.model:
                pipeline_command += ['-p', f'model_path:={args.model}']
            processes.append(subprocess.Popen(pipeline_command,
                                              stdout=logs, stderr=logs, start_new_session=True))
            command = ['ros2', 'run', 'bof_vision', 'source_node', '--ros-args', '-p', f'synthetic_color:={args.color}']
            if args.video:
                command += ['-p', 'mode:=video', '-p', f'video_path:={args.video}']
            processes.append(subprocess.Popen(command, stdout=logs, stderr=logs, start_new_session=True))
            spin_until(lambda: len(candidates) >= 10 and len(tracks) >= 10 and len(images) >= 2, 15)
            assert all(m.header.frame_id == 'camera_optical_frame' for m in candidates)
            stamps = [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in candidates]
            assert all(b > a for a,b in zip(stamps, stamps[1:])), 'nonmonotonic frame timestamps'
            assert all(m.encoding == 'bgr8' and len(m.data) == m.height * m.step for m in images)
            if args.backend in ('tongji_yolov5', 'tongji_yolov8', 'tongji_yolo11', 'shenzhen_0708', 'shenzhen_0526'):
                assert any(c.label >= 0 and 0 <= c.model_confidence <= 1 and c.geometric_score == -1
                           for m in candidates for c in m.candidates), 'no classified neural candidate received'
            if not args.video and args.backend == 'light_pair':
                assert any(m.valid and m.observed for m in tracks), 'synthetic target was not observed'
                assert all(m.status == 'tracking' and m.candidate_index == 0 and m.observed
                           for m in tracks if m.valid), 'invalid association metadata'
                assert all(m.u == 0 and m.v == 0 for m in tracks if not m.valid), 'stale pixel output'
                assert any(len(m.candidates) == 1 for m in candidates)
                stop(processes[-1])
                spin_until(lambda: not tracks[-1].valid, 3)
                assert tracks[-1].status == 'lost' and not tracks[-1].observed
                assert tracks[-1].candidate_index == -1
            assert processes[0].poll() is None, 'pipeline crashed'
            print(f'PASS: {len(candidates)} candidate messages, {len(tracks)} track messages, '
                  f'{len(images)} debug images; source={args.video or "synthetic"}')
        except Exception:
            logs.flush(); logs.seek(0)
            print(logs.read())
            raise
        finally:
            for process in reversed(processes):
                # ros2 run may have a child process; terminate only this test's own process group.
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                stop(process)
            logs.close()
            node.destroy_node()
            rclpy.shutdown()
    del subscriptions


if __name__ == '__main__':
    main()
