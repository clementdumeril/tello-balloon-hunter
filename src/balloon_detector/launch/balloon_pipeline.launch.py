#!/usr/bin/env python3
"""Launch the full balloon pipeline.

Starts, in one command:
    1. usb_cam          webcam -> /image_raw, /camera_info
    2. detector_node    YOLO detections -> /detections, /detections_image
                        (run with the yolo_env virtual environment on its
                        PYTHONPATH, after checking it is installed)
    3. balloon_pose_node  /detections + /camera_info -> /balloon_pose + TF
    4. static transform world -> camera frame, so the balloon frame is
       connected to a TF tree
    5. rviz2            preconfigured to show /detections_image, TF, and the
                        balloon pose

Before launching, the camera must be attached to WSL (a launch file cannot do
this part, it needs PowerShell):
    PowerShell (admin):  usbipd attach --wsl --busid <BUSID>
    Ubuntu:              sudo modprobe uvcvideo

Usage:
    ros2 launch balloon_detector balloon_pipeline.launch.py
    ros2 launch balloon_detector balloon_pipeline.launch.py \
        model_path:=/home/miles/balloon_data/best.pt classes:=-1 \
        balloon_diameter_m:=0.28 fallback_hfov_deg:=65.0
    ros2 launch balloon_detector balloon_pipeline.launch.py rviz:=false

Run ros2 launch --show-args balloon_detector balloon_pipeline.launch.py
for every argument.
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            Shutdown)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'balloon_detector'


def _fail(message):
    """Print an error and stop the whole launch."""
    return [LogInfo(msg=f'[balloon_pipeline] ERROR: {message}'),
            Shutdown(reason=message)]


def _truthy(text):
    return text.strip().lower() in ('true', '1', 'yes', 'on')


def _launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    # ------------------------------------------------------------------
    # Preflight 1: the camera device must exist
    # ------------------------------------------------------------------
    video_device = arg('video_device')
    if not os.path.exists(video_device):
        return _fail(
            f'{video_device} not found. Attach the camera first '
            '(PowerShell admin: usbipd attach --wsl --busid <BUSID>), then '
            'run: sudo modprobe uvcvideo')

    # ------------------------------------------------------------------
    # Preflight 2: the YOLO virtual environment must be installed
    # ------------------------------------------------------------------
    venv = os.path.expanduser(arg('venv_path'))
    site_dirs = [
        d for d in glob.glob(os.path.join(venv, 'lib', 'python3*', 'site-packages'))
        if os.path.isdir(os.path.join(d, 'ultralytics'))
    ]
    if not site_dirs:
        return _fail(
            f'ultralytics not found in the virtual environment at {venv}. '
            'Create it and install it: python3 -m venv --system-site-packages '
            f'{venv} && {venv}/bin/python -m pip install ultralytics '
            '(or pass venv_path:=<path>)')

    # Put the venv's packages on PYTHONPATH for the detector node only, ahead
    # of whatever is already set (same effect as the manual export).
    existing = os.environ.get('PYTHONPATH', '')
    pythonpath = os.pathsep.join([site_dirs[0]] + ([existing] if existing else []))

    camera_frame = arg('camera_frame')

    # ------------------------------------------------------------------
    # 1. usb_cam
    # ------------------------------------------------------------------
    usb_cam_params = {
        'video_device': video_device,
        'pixel_format': arg('pixel_format'),
        'image_width': int(arg('image_width')),
        'image_height': int(arg('image_height')),
        'framerate': float(arg('framerate')),
        'frame_id': camera_frame,
        'camera_name': 'c922',
    }
    if arg('camera_info_url'):
        usb_cam_params['camera_info_url'] = arg('camera_info_url')

    usb_cam = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
        parameters=[usb_cam_params], output='screen')

    # ------------------------------------------------------------------
    # 2. detector_node (YOLO), with the venv on its PYTHONPATH
    # ------------------------------------------------------------------
    classes = [int(c) for c in arg('classes').replace(' ', '').split(',') if c] or [-1]
    detector_params = {
        'model_path': arg('model_path'),
        'conf': float(arg('conf')),
        'imgsz': int(arg('imgsz')),
        'classes': classes,
        'publish_annotated': True,
    }
    if arg('device'):
        detector_params['device'] = arg('device')

    detector = Node(
        package=PACKAGE, executable='detector_node', name='balloon_detector',
        parameters=[detector_params], output='screen',
        additional_env={'PYTHONPATH': pythonpath})

    # ------------------------------------------------------------------
    # 3. balloon_pose_node
    # ------------------------------------------------------------------
    pose_params = {
        'balloon_diameter_m': float(arg('balloon_diameter_m')),
        'fallback_hfov_deg': float(arg('fallback_hfov_deg')),
        'camera_frame': camera_frame,
    }
    if arg('target_class'):
        pose_params['target_class'] = arg('target_class')

    pose_node = Node(
        package=PACKAGE, executable='balloon_pose_node', name='balloon_pose_node',
        parameters=[pose_params], output='screen')

    actions = [LogInfo(msg=f'[balloon_pipeline] Using YOLO environment: {site_dirs[0]}'),
               usb_cam, detector, pose_node]

    # ------------------------------------------------------------------
    # 4. static transform: world -> camera frame
    # ------------------------------------------------------------------
    # Roll/yaw of -90 degrees is the standard rotation that makes the camera
    # optical frame (z forward, x right, y down) look along world +x. Edit the
    # numbers to match how the camera is actually mounted.
    if _truthy(arg('publish_static_tf')):
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='world_to_camera',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                       '--frame-id', 'world', '--child-frame-id', camera_frame],
            output='screen'))

    # ------------------------------------------------------------------
    # 5. RViz2 with a preset layout
    # ------------------------------------------------------------------
    if _truthy(arg('rviz')):
        rviz_config = os.path.join(
            get_package_share_directory(PACKAGE), 'config', 'balloon_view.rviz')
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config], output='screen'))

    return actions


def generate_launch_description():
    def declare(name, default, description):
        return DeclareLaunchArgument(name, default_value=default, description=description)

    return LaunchDescription([
        # camera
        declare('video_device', '/dev/video0', 'Camera device'),
        declare('pixel_format', 'mjpeg2rgb', 'usb_cam pixel format'),
        declare('image_width', '320', 'Image width in pixels'),
        declare('image_height', '240', 'Image height in pixels'),
        declare('framerate', '30.0', 'Camera frame rate'),
        declare('camera_frame', 'camera_optical_frame', 'TF frame id of the camera'),
        declare('camera_info_url', '', 'Calibration file URL for usb_cam (optional)'),
        # YOLO
        declare('venv_path', os.path.expanduser('~/yolo_env'),
                'Virtual environment that has ultralytics installed'),
        declare('model_path', 'yolo11n.pt', 'YOLO weights file'),
        declare('classes', '32',
                'Comma-separated class IDs to keep; -1 = all. 32 is "sports ball" '
                'in the stock COCO model, 0 is "person"'),
        declare('conf', '0.4', 'Detection confidence threshold'),
        declare('imgsz', '320', 'YOLO inference size'),
        declare('device', '', "Inference device: '' = auto, 'cpu', or '0' for GPU"),
        # pose
        declare('balloon_diameter_m', '0.30', 'Real diameter of the balloon in meters'),
        declare('fallback_hfov_deg', '0.0',
                'Rough horizontal field of view to use if the camera is uncalibrated'),
        declare('target_class', '',
                'Only use detections with this class name (empty = any)'),
        # extras
        declare('publish_static_tf', 'true', 'Publish a static world -> camera transform'),
        declare('rviz', 'true', 'Start RViz2'),
        OpaqueFunction(function=_launch_setup),
    ])
