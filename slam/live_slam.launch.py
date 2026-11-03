"""ROS 2 launch description for a RealSense RGB-D RTAB-Map pipeline."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fps = LaunchConfiguration("fps")
    camera = Node(package="realsense2_camera", executable="realsense2_camera_node", namespace="camera", output="screen", parameters=[{
        "rgb_camera.color_profile": ["640", "480", fps],
        "depth_module.depth_profile": ["640", "480", fps],
        "align_depth.enable": True, "enable_sync": True,
        "enable_color": True, "enable_depth": True,
        "enable_infra1": False, "enable_infra2": False,
        "pointcloud.enable": False,
    }])
    odom = Node(package="rtabmap_odom", executable="rgbd_odometry", namespace="rtabmap", output="screen", parameters=[{
        "frame_id": "camera_link", "subscribe_depth": True,
        "approx_sync": False, "Odom/Strategy": "0",
    }], remappings=[("rgb/image", "/camera/camera/color/image_raw"), ("rgb/camera_info", "/camera/camera/color/camera_info"), ("depth/image", "/camera/camera/aligned_depth_to_color/image_raw")])
    mapper = Node(package="rtabmap_slam", executable="rtabmap", namespace="rtabmap", output="screen", parameters=[{
        "frame_id": "camera_link", "subscribe_depth": True,
        "approx_sync": False, "RGBD/CreateOccupancyGrid": "true",
    }], arguments=["-d"], remappings=[("rgb/image", "/camera/camera/color/image_raw"), ("rgb/camera_info", "/camera/camera/color/camera_info"), ("depth/image", "/camera/camera/aligned_depth_to_color/image_raw")])
    return LaunchDescription([DeclareLaunchArgument("fps", default_value="15"), camera, odom, mapper])
