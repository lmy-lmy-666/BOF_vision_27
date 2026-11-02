"""Compare CLI/ROS effective settings and observe the real launch's synthetic color."""
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from bof_interfaces.msg import ArmorCandidates
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image


def main():
    replay = str(Path(sys.argv[1]).resolve())
    rclpy.init()
    observer = rclpy.create_node('bof_config_observer')
    images, candidates = [], []
    subscriptions = [
        observer.create_subscription(Image, '/camera/image_raw', images.append, qos_profile_sensor_data),
        observer.create_subscription(ArmorCandidates, '/perception/candidates', candidates.append, qos_profile_sensor_data),
    ]
    try:
        with tempfile.TemporaryDirectory(prefix='bof-config-ros-') as temporary:
            directory = Path(temporary)
            config = directory / 'blue.yaml'
            config.write_text('enemy_color: blue\ntrack_timeout_s: 0.35\n'
                              'association_gate_px: 60.0\nnms_threshold: 0.4\n')
            for override in (None, 'red'):
                images.clear(); candidates.clear()
                color = override or 'blue'
                cli_snapshot = directory / f'cli-{color}.yaml'
                ros_snapshot = directory / f'ros-{color}.yaml'
                cli = [replay, '--config', str(config), '--dump-config', str(cli_snapshot), '--check-config']
                launch = ['ros2', 'launch', 'bof_bringup', 'vision.launch.py',
                          f'config:={config}', f'effective_config:={ros_snapshot}']
                if override:
                    cli += ['--color', override]
                    launch += [f'enemy_color:={override}']
                subprocess.run(cli, check=True, capture_output=True, timeout=20)
                with (directory / f'launch-{color}.log').open('w+') as log:
                    process = subprocess.Popen(launch, stdout=log, stderr=log, start_new_session=True)
                    try:
                        deadline = time.monotonic() + 20
                        while time.monotonic() < deadline:
                            rclpy.spin_once(observer, timeout_sec=0.05)
                            if ros_snapshot.exists() and len(images) >= 10 and len(candidates) >= 10:
                                break
                        assert process.poll() is None, 'launch exited early'
                        assert len(images) >= 10 and len(candidates) >= 10, 'no launched pipeline output'
                        assert yaml.safe_load(cli_snapshot.read_text()) == yaml.safe_load(ros_snapshot.read_text())
                        assert any(m.candidates for m in candidates), 'synthetic color and detector disagree'
                        frame = images[-1]
                        assert frame.encoding == 'bgr8' and frame.step == frame.width * 3
                        blue, red = sum(frame.data[0::3]), sum(frame.data[2::3])
                        assert (blue > red) if color == 'blue' else (red > blue), 'wrong source color'
                        client = observer.create_client(SetParameters, '/pipeline/set_parameters')
                        try:
                            assert client.wait_for_service(timeout_sec=10), 'parameter service missing'
                            request = SetParameters.Request()
                            request.parameters = [Parameter('enemy_color', value='blue' if color == 'red' else 'red').to_parameter_msg()]
                            future = client.call_async(request)
                            rclpy.spin_until_future_complete(observer, future, timeout_sec=10)
                            assert future.done(), 'parameter service timed out'
                            result = future.result().results[0]
                            assert not result.successful and 'read-only' in result.reason, result.reason
                        finally:
                            observer.destroy_client(client)
                        print(f'PASS: launch color={color}, CLI/ROS snapshots equal, startup-only parameters enforced')
                    except Exception:
                        log.flush(); log.seek(0)
                        print(log.read())
                        raise
                    finally:
                        if process.poll() is None:
                            os.killpg(process.pid, signal.SIGINT)
                            try:
                                process.wait(timeout=8)
                            except subprocess.TimeoutExpired:
                                os.killpg(process.pid, signal.SIGKILL)
                                process.wait()
                # Drain queued observations before starting the next independent launch.
                deadline = time.monotonic() + 0.3
                while time.monotonic() < deadline:
                    rclpy.spin_once(observer, timeout_sec=0.05)
    finally:
        observer.destroy_node()
        rclpy.shutdown()
    del subscriptions


if __name__ == '__main__':
    main()
