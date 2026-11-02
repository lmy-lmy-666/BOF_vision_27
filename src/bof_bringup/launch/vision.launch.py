"""Only launch wiring is Python; all processing and camera code is C++."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    mode = value('source')
    if mode not in ('synthetic', 'video', 'external', 'hik'):
        raise ValueError('source must be synthetic, video, external or hik')
    string = lambda text: ParameterValue(text, value_type=str)
    if value('respawn').lower() not in ('true', 'false'):
        raise ValueError('respawn must be true or false')
    # Match mode: restart a crashed node after 1 s (Tongji watchdog.sh equivalent).
    respawn = {'respawn': value('respawn').lower() == 'true', 'respawn_delay': 1.0}
    parameters = {'config_file': string(value('config')),
                  'effective_config_path': string(value('effective_config'))}
    # Only explicit arguments override YAML. Empty defaults mean "use the file".
    for argument, parameter in [('enemy_color', 'enemy_color'), ('backend', 'detector_backend'),
                                ('model', 'model_path'), ('device', 'inference_device'), ('yolo', 'yolo_name')]:
        if value(argument):
            parameters[parameter] = string(value(argument))
    nodes = [Node(package='bof_vision', executable='pipeline_node', name='pipeline',
                  parameters=[parameters],
                  remappings=[('camera/image_raw', value('image_topic'))], output='screen', **respawn)]
    if mode in ('synthetic', 'video'):
        source_parameters = {'mode': mode, 'config_file': string(value('config'))}
        if value('video_path'):
            source_parameters['video_path'] = string(value('video_path'))
        if value('fps'):
            source_parameters['fps'] = float(value('fps'))
        if value('enemy_color'):
            source_parameters['synthetic_color'] = string(value('enemy_color'))
        nodes.append(Node(package='bof_vision', executable='source_node', name='source',
                          parameters=[source_parameters],
                          remappings=[('camera/image_raw', value('image_topic'))], output='screen'))
    elif mode == 'hik':
        camera_parameters = {'config_file': string(value('config'))}
        if value('camera_serial'):
            camera_parameters['serial_number'] = string(value('camera_serial'))
        for key in ('exposure_us', 'gain'):
            if value(key):
                camera_parameters[key] = float(value(key))
        nodes.append(Node(package='bof_vision', executable='hik_camera_node', name='hik_camera',
                          parameters=[camera_parameters],
                          remappings=[('camera/image_raw', value('image_topic'))], output='screen', **respawn))
    if value("aim").lower() == "true":
        nodes.append(Node(package="bof_vision", executable="aim_node", name="aim",
                          parameters=[{"config_file": string(value("config"))}], output="screen", **respawn))
    elif value("aim").lower() != "false":
        raise ValueError("aim must be true or false")
    return nodes


def generate_launch_description():
    config = str(Path(get_package_share_directory('bof_bringup')) / 'config' / 'vision.yaml')
    args = {'source': 'synthetic', 'video_path': '', 'fps': '', 'enemy_color': '',
            'config': config, 'image_topic': '/camera/image_raw', 'camera_serial': '', 'exposure_us': '', 'gain': '',
            'aim': 'false', 'respawn': 'false', 'backend': '', 'model': '', 'device': '', 'yolo': '', 'effective_config': ''}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in args.items()]
                             + [OpaqueFunction(function=setup)])
