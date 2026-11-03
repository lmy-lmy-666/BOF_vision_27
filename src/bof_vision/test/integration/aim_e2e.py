"""End-to-end check: fake gimbal C board (pty, team firmware cboard frames) + synthetic spinning outpost ArmorPoses -> aim_node.

Usage (after source src/scripts/env.sh): python3 aim_e2e.py OUTPUT_DIR src/bof_bringup/config/sentry.yaml
The fake gimbal jumps to each commanded angle; poses are projected through that attitude.
After 5 s a directive marks the outpost invincible, which must release the target.
Simulation timestamps only; this is not a hardware or protocol acceptance."""
import math, os, struct, subprocess, sys, time, threading, tty
import numpy as np, cv2, yaml, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from bof_interfaces.msg import ArmorPoses, ArmorPose, AutoAimTarget, AimSolution, AimDirective, GimbalState, GimbalCommand
from sensor_msgs.msg import Imu, Image

def crc16(data):
    # Same reflected 0x8408 polynomial as the SP table.
    crc = 0xffff
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc

out_dir = sys.argv[1]
master, slave = os.openpty(); tty.setraw(master)
port = os.ttyname(slave)
base = yaml.safe_load(open(sys.argv[2]))
aim = base['aim']
aim.update(enabled=True, timestamp_source='simulation')
aim['gimbal_link'].update(imu_source='serial', serial_port=port, protocol='cboard')
aim['extrinsics'] = dict(camera_to_gimbal_q_xyzw=[0.5, -0.5, 0.5, -0.5], camera_in_gimbal_m=[0, 0, 0],
                         muzzle_in_gimbal_m=[0, 0, 0])
aim['aimer']['muzzle_speed_mps'] = 22.0
aim['shooter']['auto_fire'] = True
aim['decision']['enabled'] = True
aim['debug_image_rate_hz'] = 20.0
camera_yaml = os.path.join(out_dir, 'e2e_camera.yaml')
yaml.safe_dump({'image_width': 640, 'image_height': 480, 'camera_matrix': [500, 0, 320, 0, 500, 240, 0, 0, 1],
                'distort_coeffs': [0, 0, 0, 0, 0]}, open(camera_yaml, 'w'))
base['calibration_path'] = camera_yaml
cfg = os.path.join(out_dir, 'aim_e2e.yaml'); yaml.safe_dump(base, open(cfg, 'w'), allow_unicode=True)
proc = subprocess.Popen(['ros2', 'run', 'bof_vision', 'aim_node', '--ros-args', '-p', f'config_file:={cfg}'],
                        stdout=open(os.path.join(out_dir, 'aim_node.log'), 'w'), stderr=subprocess.STDOUT, start_new_session=True)
stop = False
sent_modes = {}
cmd = [0.0, 0.0]
def serial_io():
    buf = b''
    while not stop:
        y, pt = cmd  # pitch up positive; quaternion of yaw about z then pitch about -y
        cy, sy, cp, sp_ = math.cos(y/2), math.sin(y/2), math.cos(-pt/2), math.sin(-pt/2)
        q = (cy*cp, -sy*sp_, cy*sp_, sy*cp)
        # Firmware Auto_aim_TX: 0x53, mode 1, enemy_color, q wxyz, yaw, yaw_vel, pitch = INS.Roll (nose down), ...
        frame = struct.pack('<BBB4f5fH', 0x53, 1, 1, *q, y, 0, -pt, 0, 22.0, 1)
        frame += struct.pack('<H', crc16(frame))
        os.write(master, frame)
        try:
            os.set_blocking(master, False); buf += os.read(master, 4096)
        except BlockingIOError:
            pass
        while len(buf) >= 28:
            i = buf.find(b'\x50')
            if i < 0: buf = b''; break
            buf = buf[i:]
            if len(buf) < 28: break
            f = buf[:28]; buf = buf[28:]
            if crc16(f[:-2]) != struct.unpack('<H', f[-2:])[0]: sent_modes['bad_crc'] = sent_modes.get('bad_crc', 0) + 1; continue
            mode = f[1]; sent_modes[mode] = sent_modes.get(mode, 0) + 1
            # Gimbol_AUTO: INS.Yaw -> yaw, INS.Roll -> -pitch, so the command pitch is elevation.
            if mode: yaw, _, _, pitch = struct.unpack('<4f', f[2:18]); cmd[:] = [yaw, pitch]
        time.sleep(0.001)
threading.Thread(target=serial_io, daemon=True).start()

rclpy.init(); node = Node('aim_e2e')
pub = node.create_publisher(ArmorPoses, 'perception/armor_poses', qos_profile_sensor_data)
directive = node.create_publisher(AimDirective, 'sentry/aim_directive', 10)
targets, solutions = [], []
node.create_subscription(AutoAimTarget, 'auto_aim/target', targets.append, 10)
node.create_subscription(AimSolution, 'aim/solution', solutions.append, 10)
io_counts = {'imu': 0, 'state': 0, 'command': 0}
def counter(key):
    def f(_): io_counts[key] += 1
    return f
node.create_subscription(Imu, 'gimbal/imu', counter('imu'), qos_profile_sensor_data)
debug_pub = node.create_publisher(Image, 'debug/image', qos_profile_sensor_data)
overlays = []
node.create_subscription(Image, 'aim/debug_image', overlays.append, qos_profile_sensor_data)
node.create_subscription(GimbalState, 'gimbal/state', counter('state'), qos_profile_sensor_data)
node.create_subscription(GimbalCommand, 'gimbal/command', counter('command'), qos_profile_sensor_data)
# Gimbal frame: x forward, y left, z up. Optical: x right, y down, z forward.
R_go = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
w, r, center = -0.8 * math.pi, 0.2765, np.array([5.0, 0.3, 0.4])
heights = [0.0, 0.1, -0.1]
start = time.time(); time.sleep(1.0)
while time.time() - start < 7.0:
    t = time.time() - start
    msg = ArmorPoses(); msg.header.stamp = node.get_clock().now().to_msg(); msg.header.frame_id = 'camera_optical_frame'
    bearing = math.atan2(center[1], center[0])
    y, pt = cmd
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    Ry = np.array([[math.cos(-pt), 0, math.sin(-pt)], [0, 1, 0], [-math.sin(-pt), 0, math.cos(-pt)]])
    Rg = Rz @ Ry   # gimbal -> reference at publish time
    for k in range(3):
        a = w * t + k * 2 * math.pi / 3
        if abs(math.remainder(a - bearing, 2 * math.pi)) > 0.9: continue
        p = center + np.array([-r * math.cos(a), -r * math.sin(a), heights[k]])
        normal = np.array([math.cos(a), math.sin(a), 0.0])        # inward, gimbal frame
        right = np.array([math.sin(a), -math.cos(a), 0.0]); down = np.array([0, 0, -1.0])
        R_plate = R_go.T @ Rg.T @ np.column_stack([right, down, normal])  # plate -> optical
        rv, _ = cv2.Rodrigues(R_plate)
        o = R_go.T @ Rg.T @ p
        pose = ArmorPose(); pose.label = 6; pose.valid = True
        pose.position_m.x, pose.position_m.y, pose.position_m.z = map(float, o)
        pose.rotation_vector.x, pose.rotation_vector.y, pose.rotation_vector.z = map(float, rv.ravel())
        msg.poses.append(pose)
    pub.publish(msg)
    img = Image(); img.header = msg.header; img.height, img.width, img.encoding, img.step = 480, 640, 'bgr8', 1920
    img.data = bytes(640 * 480 * 3); debug_pub.publish(img)
    if t > 5.0:
        d = AimDirective(); d.header.stamp = msg.header.stamp; d.aim_enable = True; d.fire_enable = True
        d.excluded_labels = [6]; directive.publish(d)
    rclpy.spin_once(node, timeout_sec=0.005)
    time.sleep(0.005)
for _ in range(50): rclpy.spin_once(node, timeout_sec=0.01)
stop = True; import signal; os.killpg(proc.pid, signal.SIGINT); print('node exit code', proc.wait(10))
before = [m for m in targets if m.header.stamp.sec and m.valid]
last_valid = before[-1] if before else None
sol_valid = [s for s in solutions if s.valid]
print('targets', len(targets), 'valid', len(before), 'solutions_valid', len(sol_valid), 'serial modes', sent_modes)
if last_valid:
    print('last valid: spin=%s v_yaw=%.3f pos=(%.3f,%.3f,%.3f) state=%s' % (last_valid.spin, last_valid.v_yaw,
          last_valid.position.x, last_valid.position.y, last_valid.position.z, last_valid.tracking_state))
print('final target state:', targets[-1].tracking_state if targets else None, 'valid', targets[-1].valid if targets else None)
statuses = {}
for s in solutions: statuses[s.fire_status] = statuses.get(s.fire_status, 0) + 1
print('fire statuses', statuses)
print('gimbal io topics received', io_counts)
drawn = [m for m in overlays if any(m.data[i] for i in range(0, len(m.data), 97))]
print('aim debug images', len(overlays), 'with drawing', len(drawn))
if drawn:
    pick = drawn[len(drawn) // 2]
    cv2.imwrite(os.path.join(out_dir, 'aim_debug_sample.png'), np.frombuffer(pick.data, np.uint8).reshape(480, 640, 3))

prev=None
for m in targets:
    if m.tracking_state!=prev:
        print('%.3f %s valid=%s id=%d'%(m.header.stamp.sec+m.header.stamp.nanosec*1e-9-start, m.tracking_state, m.valid, m.track_id)); prev=m.tracking_state
