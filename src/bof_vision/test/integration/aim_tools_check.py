"""Synthetic rosbags with known answers, then bof_handeye / bof_aim_replay checked against them.

Usage (after source scripts/env.sh): python3 aim_tools_check.py OUTPUT_DIR src/bof_bringup/config/sentry.yaml
Writes OUTPUT_DIR/{outpost,static,handeye}_bag, camera.yaml, truth.yaml and the tool outputs.
Simulation only: proves the tools recover known parameters, not hardware accuracy.
"""
import math, os, re, shutil, subprocess, sys
import numpy as np, cv2, yaml
import rclpy.serialization
from rclpy.time import Time
import rosbag2_py
from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import Imu, Image
from bof_interfaces.msg import ArmorPoses, ArmorPose, GimbalCommand, GimbalState

OUT = sys.argv[1]
T0 = 1_700_000_000 * 10**9
CAMERA_LATENCY = 0.012       # pose stamp = exposure + this
GIMBAL_LAG = 0.030           # gimbal yaw follows the command this much later
FIRE_TO_COUNT = 0.050        # bullet_count increments this long after a fire edge
# Camera optical -> gimbal: ideal mount rotated 1.5 deg in yaw, 1 deg in pitch, plus an offset.
R_IDEAL = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
def rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    return {'z': np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]), 'y': np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])}[axis]
R_CAM = rot('z', math.radians(1.5)) @ rot('y', math.radians(-1.0)) @ R_IDEAL
T_CAM = np.array([0.08, -0.02, 0.05])
K = np.array([[1000, 0, 640], [0, 1000, 512], [0, 0, 1]], float)
SIZE = (1280, 1024)

def quat_xyzw(R):
    w = math.sqrt(max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    return [(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w]
def stamp(ns):
    return TimeMsg(sec=ns // 10**9, nanosec=ns % 10**9)
def gimbal_R(yaw, pitch):
    return rot('z', yaw) @ rot('y', -pitch)
def writer(name):
    path = os.path.join(OUT, name)
    shutil.rmtree(path, ignore_errors=True)
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    topics = {}
    def write(topic, msg, ns):
        if topic not in topics:
            w.create_topic(rosbag2_py.TopicMetadata(id=len(topics), name=topic, type=type(msg).__module__.split('.')[0] + '/msg/' + type(msg).__name__,
                                                    serialization_format='cdr'))
            topics[topic] = True
        w.write(topic, rclpy.serialization.serialize_message(msg), ns)
    return w, write
def imu_msg(ns, R):
    m = Imu(); m.header.stamp = stamp(ns); m.header.frame_id = 'gimbal'
    x, y, z, w = quat_xyzw(R); m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = x, y, z, w
    return m
def pose_of(label, p_world, normal_world, Rg):
    """World plate centre + inward normal -> optical ArmorPose for gimbal attitude Rg."""
    right = np.array([normal_world[1], -normal_world[0], 0.0]); right /= np.linalg.norm(right)
    down = np.array([0, 0, -1.0])
    R_plate = (Rg @ R_CAM).T @ np.column_stack([right, down, normal_world])
    o = R_CAM.T @ (Rg.T @ p_world - T_CAM)
    rv, _ = cv2.Rodrigues(R_plate)
    a = ArmorPose(); a.label = label; a.valid = True
    a.position_m.x, a.position_m.y, a.position_m.z = map(float, o)
    a.rotation_vector.x, a.rotation_vector.y, a.rotation_vector.z = map(float, rv.ravel())
    return a

def gimbal_motion(t):
    return 0.15 * math.sin(2 * math.pi * 0.5 * t), 0.05 * math.sin(2 * math.pi * 0.7 * t)

def outpost_bag():
    """Spinning outpost, swinging gimbal, commands, fire edges and bullet counts."""
    w, write = writer('outpost_bag')
    centre, r, omega = np.array([5.0, 0.3, 0.4]), 0.2765, -0.8 * math.pi
    heights = [0.0, 0.1, -0.1]
    count, next_fire, pending_counts = 0, 1.0, []
    for i in range(int(8.0 * 1000)):            # 1 kHz attitude and command stream
        t = i / 1000.0; ns = T0 + i * 10**6
        cmd_yaw, cmd_pitch = gimbal_motion(t)
        yaw, pitch = gimbal_motion(t - GIMBAL_LAG)
        write('/gimbal/imu', imu_msg(ns, gimbal_R(yaw, pitch)), ns)
        fire = t >= next_fire and t < next_fire + 0.02
        if t >= next_fire + 0.02:
            pending_counts.append(next_fire + FIRE_TO_COUNT); next_fire += 0.25
        if pending_counts and t >= pending_counts[0]:
            count += 1; pending_counts.pop(0)
        c = GimbalCommand(); c.header.stamp = stamp(ns); c.control = True; c.fire = bool(fire); c.yaw = cmd_yaw; c.pitch = cmd_pitch
        write('/gimbal/command', c, ns)
        s = GimbalState(); s.header.stamp = stamp(ns); s.mode = 1; s.bullet_speed_mps = 23.0; s.bullet_count = count
        write('/gimbal/state', s, ns)
        if i % 10 == 0:                          # 100 Hz camera
            exposure = ns; Rg = gimbal_R(yaw, pitch)
            m = ArmorPoses(); m.header.stamp = stamp(exposure + int(CAMERA_LATENCY * 1e9)); m.header.frame_id = 'camera_optical_frame'
            bearing = math.atan2(centre[1], centre[0])
            for k in range(3):
                a = omega * t + k * 2 * math.pi / 3
                if abs(math.remainder(a - bearing, 2 * math.pi)) > 0.9: continue
                p = centre + np.array([-r * math.cos(a), -r * math.sin(a), heights[k]])
                m.poses.append(pose_of(6, p, np.array([math.cos(a), math.sin(a), 0.0]), Rg))
            write('/perception/armor_poses', m, exposure + int((CAMERA_LATENCY + 0.004) * 1e9))
    del w

def static_bag():
    """Static plate, swinging gimbal: for the camera latency sweep."""
    w, write = writer('static_bag')
    p = np.array([4.0, 0.2, 0.3]); normal = np.array([1.0, 0.05, 0.0]); normal /= np.linalg.norm(normal)
    for i in range(6000):
        t = i / 1000.0; ns = T0 + i * 10**6
        yaw, pitch = gimbal_motion(t); Rg = gimbal_R(yaw, pitch)
        write('/gimbal/imu', imu_msg(ns, Rg), ns)
        if i % 10 == 0:
            m = ArmorPoses(); m.header.stamp = stamp(ns + int(CAMERA_LATENCY * 1e9)); m.header.frame_id = 'camera_optical_frame'
            m.poses.append(pose_of(3, p, normal, Rg))
            write('/perception/armor_poses', m, ns + int((CAMERA_LATENCY + 0.004) * 1e9))
    del w

def handeye_bag():
    """Fixed circle board, gimbal held still at a grid of yaw/pitch, moving between stops."""
    w, write = writer('handeye_bag')
    cols, rows, spacing = 7, 5, 0.03
    # Board plane faces the gimbal: board x -> world -y, board y -> world -z, board z -> world +x (right-handed).
    board_R = np.column_stack([[0, -1, 0], [0, 0, -1], [1, 0, 0]]).astype(float)
    board_t = np.array([1.2, 0.09, 0.06])
    stops = [(y, p) for y in (-0.25, 0.0, 0.25) for p in (-0.2, 0.0, 0.2)] + [(0.12, 0.1), (-0.12, -0.1), (0.0, 0.3)]
    obj = np.array([[c * spacing, r * spacing, 0] for r in range(rows) for c in range(cols)], float)
    ns = T0; prev = (0.0, 0.0)
    for stop in stops:
        for j in range(500):                       # 0.5 s move, then 1 s still
            a = min(1.0, j / 500.0); yaw = prev[0] + (stop[0] - prev[0]) * a; pitch = prev[1] + (stop[1] - prev[1]) * a
            write('/gimbal/imu', imu_msg(ns, gimbal_R(yaw, pitch)), ns); ns += 10**6
        for j in range(1000):
            Rg = gimbal_R(*stop)
            write('/gimbal/imu', imu_msg(ns, Rg), ns)
            if j in (500,):
                R_bc = (Rg @ R_CAM).T @ board_R; t_bc = (Rg @ R_CAM).T @ board_t - R_CAM.T @ T_CAM
                pts, _ = cv2.projectPoints(obj, cv2.Rodrigues(R_bc)[0], t_bc, K, None)
                img = np.full((SIZE[1], SIZE[0], 3), 255, np.uint8)
                radius = int(1000 * 0.008 / t_bc[2])
                for q in pts.reshape(-1, 2):   # sub-pixel centres (shift=4)
                    cv2.circle(img, (int(round(q[0] * 16)), int(round(q[1] * 16))), radius * 16, (0, 0, 0), -1, cv2.LINE_AA, 4)
                m = Image(); m.header.stamp = stamp(ns); m.header.frame_id = 'camera_optical_frame'
                m.height, m.width, m.encoding, m.step = SIZE[1], SIZE[0], 'bgr8', SIZE[0] * 3; m.data = img.tobytes()
                write('/camera/image_raw', m, ns)
            ns += 10**6
        prev = stop
    del w

os.makedirs(OUT, exist_ok=True)
outpost_bag(); static_bag(); handeye_bag()
yaml.safe_dump({'image_width': SIZE[0], 'image_height': SIZE[1], 'distortion_model': 'opencv_pinhole',
                'camera_matrix': K.ravel().tolist(), 'distort_coeffs': [0.0] * 5}, open(os.path.join(OUT, 'camera.yaml'), 'w'))
yaml.safe_dump({'camera_latency_s': CAMERA_LATENCY, 'gimbal_lag_s': GIMBAL_LAG, 'fire_to_count_s': FIRE_TO_COUNT,
                'camera_to_gimbal_q_xyzw': [float(v) for v in quat_xyzw(R_CAM)], 'camera_in_gimbal_m': T_CAM.tolist(),
                'outpost_spin': 'clockwise', 'outpost_height_steps_m': [0.0, 0.1, -0.1]}, open(os.path.join(OUT, 'truth.yaml'), 'w'))
print('wrote synthetic bags to', OUT)

def run(*args):
    r = subprocess.run(['ros2', 'run', 'bof_vision', *args], capture_output=True, text=True)
    print(r.stdout + r.stderr)
    if r.returncode: sys.exit('tool failed: ' + ' '.join(args))
    return r.stdout
failures = []
def check(name, value, expected, tolerance):
    ok = abs(value - expected) <= tolerance
    print(f"CHECK {name}: {value:.5f} expected {expected:.5f} +- {tolerance} -> {'PASS' if ok else 'FAIL'}")
    if not ok: failures.append(name)

handeye = os.path.join(OUT, 'handeye.yaml')
run('bof_handeye', '--bag', os.path.join(OUT, 'handeye_bag'), '--calibration', os.path.join(OUT, 'camera.yaml'),
    '--pattern', 'circles', '--cols', '7', '--rows', '5', '--spacing-mm', '30', '--min-views', '8', '--output', handeye)
result = yaml.safe_load(open(handeye))
q_true = np.array(quat_xyzw(R_CAM)); q = np.array(result['camera_to_gimbal_q_xyzw'])
check('handeye rotation error deg', math.degrees(2 * math.acos(min(1.0, abs(float(q @ q_true))))), 0, 0.1)
check('handeye translation error m', float(np.linalg.norm(np.array(result['camera_in_gimbal_m']) - T_CAM)), 0, 0.005)

# Aim config: shipped sentry.yaml with the recovered extrinsics and simulated timing.
base = yaml.safe_load(open(sys.argv[2])); aim = base['aim']
aim['extrinsics'] = {'camera_to_gimbal_q_xyzw': result['camera_to_gimbal_q_xyzw'],
                     'camera_in_gimbal_m': result['camera_in_gimbal_m'], 'muzzle_in_gimbal_m': [0, 0, 0]}
aim['aimer']['muzzle_speed_mps'] = 23.0
cfg = os.path.join(OUT, 'aim.yaml'); yaml.safe_dump(base, open(cfg, 'w'), allow_unicode=True)
out = run('bof_aim_replay', '--bag', os.path.join(OUT, 'static_bag'), '--config', cfg, '--mode', 'latency',
          '--min', '0', '--max', '0.03', '--step', '0.001')
check('camera latency s', float(re.search(r'best camera_latency_s ~= ([0-9.]+)', out).group(1)), CAMERA_LATENCY, 0.0015)
out = run('bof_aim_replay', '--bag', os.path.join(OUT, 'outpost_bag'), '--config', cfg, '--mode', 'command_lag')
check('gimbal lag s', float(re.search(r'lag ~= ([0-9.]+)', out).group(1)), GIMBAL_LAG, 0.0015)
check('fire to count s', float(re.search(r'median fire->count delay ([0-9.]+)', out).group(1)), FIRE_TO_COUNT, 0.0015)

aim['timestamp_source'] = 'host_compensated'; aim['camera_latency_s'] = CAMERA_LATENCY
yaml.safe_dump(base, open(cfg, 'w'), allow_unicode=True)
out = run('bof_aim_replay', '--bag', os.path.join(OUT, 'outpost_bag'), '--config', cfg, '--mode', 'replay',
          '--output', os.path.join(OUT, 'replay'))
if 'spin clockwise' not in out: failures.append('outpost spin'); print('CHECK outpost spin -> FAIL')
else: print('CHECK outpost spin clockwise -> PASS')
for plate, mean in re.findall(r'outpost plate (\d) z residual: mean ([-0-9.e]+)', out):
    check(f'outpost plate {plate} z residual mean m', float(mean), 0, 0.01)
if len(re.findall(r'outpost plate \d z residual', out)) != 3: failures.append('three plate residuals')
print('FAILED: ' + ', '.join(failures) if failures else 'ALL CHECKS PASS')
sys.exit(1 if failures else 0)
