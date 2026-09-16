"""
Vision-specific launch helpers.

Generic helpers (safe_params, env_truthy) live in narsil_bringup_common
and are imported via the normal Python import path. This module keeps the
narrowly vision-flavoured stuff: camera profile resolution, calibration
URL building, narsil_bringup_vision config loader.

The calibration directory is mounted by docker/vision.Dockerfile at
CALIB_DIR_IN_CONTAINER — see deploy/bee/compose.yml volumes.
"""

import json
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode

from narsil_bringup_common import env_truthy, safe_params

# Matches the volume mount in deploy/bee/compose.yml:
#   ./calibration/common:/opt/narsil_ws/calibration:ro
CALIB_DIR_IN_CONTAINER = '/opt/narsil_ws/calibration'


def load_config(name):
    """Load narsil_bringup_vision/config/<name>.yaml as a dict."""
    path = os.path.join(
        get_package_share_directory('narsil_bringup_vision'),
        'config', f'{name}.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


def active_model_manifest():
    """Return (model_name, manifest dict) for the model NARSIL_VISION_MODEL selects."""
    name = os.environ.get('NARSIL_VISION_MODEL', 'yolov8_drone')
    path = os.path.join(
        get_package_share_directory('narsil_bringup_vision'),
        'config', 'models', name, 'manifest.json')
    with open(path) as f:
        return name, json.load(f)


def model_network_size(manifest=None):
    """Return the model's (width, height) input in pixels.

    The decoder emits boxes in this letterbox space, so every consumer that
    un-letterboxes MUST scale by these. A stale default silently rescales:
    640 against a 1280-wide network halved the tracker's reported range.
    """
    if manifest is None:
        _, manifest = active_model_manifest()
    shape = manifest.get(
        'input_shape', [1, 3, int(manifest['imgsz']), int(manifest['imgsz'])])
    return float(shape[3]), float(shape[2])


def active_camera_profile(cam_cfg=None):
    """
    Resolve the active CSI camera profile.

    Priority: CAMERA_PROFILE env var > camera_csi.yaml's default_profile.
    Returns (profile_name, profile_dict). Raises on unknown profile.
    """
    if cam_cfg is None:
        cam_cfg = load_config('camera_csi')
    name = os.environ.get('CAMERA_PROFILE') or cam_cfg.get('default_profile')
    if not name:
        raise ValueError(
            'No camera profile selected. Set CAMERA_PROFILE env var or '
            'define default_profile in config/camera_csi.yaml.')
    profiles = cam_cfg.get('profiles', {})
    if name not in profiles:
        raise KeyError(
            f"Camera profile '{name}' not found. "
            f"Available: {sorted(profiles)}")
    return name, profiles[name]


def calibration_url(profile, profile_name):
    """
    Build the file:// URL for a profile's calibration YAML.

    Fails fast at launch time if the file is missing rather than letting
    argus_mono abort later with a generic camera_info_manager error.
    """
    path = os.path.join(CALIB_DIR_IN_CONTAINER, profile['calibration_file'])
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"Profile '{profile_name}' needs calibration at {path}, "
            f"but it's missing.\n"
            f"  Mount the calibration directory at {CALIB_DIR_IN_CONTAINER}\n"
            f"  (see deploy/bee/compose.yml calibration volume).")
    return f'file://{path}'


def camera_nodes(profile, profile_name):
    """
    Camera source for the detection graph, selected by profile['driver']
    (default argus). Shared by both backend launches so the camera choice
    stays orthogonal to the backend choice.

    Empty under NARSIL_VISION_REPLAY: a played bag of /left/image_raw +
    /left/camera_info feeds the encoder instead.
    """
    if env_truthy('NARSIL_VISION_REPLAY'):
        return []

    if profile.get('driver', 'argus') == 'v4l2':
        right_file = profile.get('calibration_file_right')
        right_url = (calibration_url({'calibration_file': right_file},
                                     profile_name) if right_file else None)
        capture = ComposableNode(
            name='ar0234_camera',
            package='narsil_ar0234_camera',
            plugin='narsil_ar0234_camera::ArducamNode',
            namespace='ar0234',
            parameters=[safe_params({
                **load_config('ar0234'),
                'video_device': profile['device'],
                'image_width': profile['capture_width'],
                'image_height': profile['capture_height'],
                'framerate': profile.get('framerate', 30),
                'left_camera_info_url': calibration_url(profile, profile_name),
                'right_camera_info_url': right_url,
                'left_camera_name': profile.get('camera_name'),
                'right_camera_name': profile.get('camera_name_right'),
            }, required=['video_device', 'left_camera_info_url'])],
            remappings=[('left/camera_info', '/left/camera_info')],
        )
        # GPU debayer, left eye only (the detector input): the canonical
        # /left/image_raw is the debayered rgb8, not the namespaced bayer.
        debayer = ComposableNode(
            name='ar0234_left_debayer',
            package='narsil_ar0234_camera',
            plugin='narsil_ar0234_camera::GpuDebayerNode',
            namespace='ar0234',
            parameters=[safe_params({
                'conformance_check': env_truthy('NARSIL_AR0234_CONFORMANCE'),
            })],
            remappings=[
                ('image_raw', 'left/image_raw'),
                ('image_color', '/left/image_raw'),
                ('preview/compressed', '/vision/image_small/compressed'),
            ],
        )
        nodes = [capture, debayer]
        if env_truthy('NARSIL_FLOW_ODOMETRY'):
            # NARSIL_FLOW_STATIC_ATTITUDE=1 (twin env, FC-less bench rigs):
            # identity derotation — see flow_odometry.yaml.
            flow_params = dict(load_config('flow_odometry'))
            flow_params['bee_id'] = int(os.environ.get('NARSIL_BEE_ID', '0'))
            if env_truthy('NARSIL_FLOW_STATIC_ATTITUDE'):
                flow_params['static_attitude_mode'] = True
            # NARSIL_FLOW_DEBUG bitmask: 1 = debug image, 2 = path compare.
            if int(os.environ.get('NARSIL_FLOW_DEBUG', '0') or '0') & 1:
                flow_params['debug_image'] = True
            nodes.append(ComposableNode(
                name='flow_odometry',
                package='narsil_flow_odometry',
                plugin='narsil_flow_odometry::FlowOdometryNode',
                namespace='ar0234',
                parameters=[safe_params(flow_params)],
                remappings=[
                    ('left/camera_info', '/left/camera_info'),
                    ('narsil/babel/attitude',
                     f"/bee_{flow_params['bee_id']}/narsil/babel/attitude"),
                ],
            ))
        return nodes

    return [ComposableNode(
        name='argus_mono',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        namespace='',
        parameters=[safe_params({
            'module_id': profile['module_id'],
            'mode': profile['sensor_mode'],
            'camera_info_url': calibration_url(profile, profile_name),
        }, required=['camera_info_url'])],
    )]
