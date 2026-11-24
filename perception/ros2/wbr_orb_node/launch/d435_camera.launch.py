"""D435 IR stereo + raw depth, or RGB + aligned depth. No camera IMU."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def camera_parameters(mode, serial, width, height, fps):
    if mode not in ("stereo", "rgbd"):
        raise ValueError("mode must be stereo or rgbd")
    if not serial or not serial.isdecimal():
        raise ValueError("serial must be the actual camera serial")
    if min(int(width), int(height), int(fps)) <= 0:
        raise ValueError("width, height and fps must be positive")
    profile = f"{int(width)},{int(height)},{int(fps)}"
    return {
        "camera_name": "camera", "serial_no": serial,
        "enable_infra1": mode == "stereo", "enable_infra2": mode == "stereo",
        "enable_color": mode == "rgbd",
        "enable_depth": True, "enable_gyro": False, "enable_accel": False,
        "depth_module.infra_profile": profile, "depth_module.depth_profile": profile,
        "rgb_camera.color_profile": profile,
        "depth_module.infra1_format": "Y8", "depth_module.infra2_format": "Y8",
        "depth_module.depth_format": "Z16", "rgb_camera.color_format": "RGB8",
        "enable_sync": True, "align_depth.enable": mode == "rgbd",
        "pointcloud.enable": False,
        # Projected texture moves with camera and harms stereo feature matching.
        # For RGB-D allow the emitter to support the depth sensor.
        "depth_module.emitter_enabled": 0 if mode == "stereo" else 1,
    }


def setup(context):
    args = camera_parameters(*(LaunchConfiguration(key).perform(context)
                               for key in ("mode", "serial", "width", "height", "fps")))
    args["serial_no"] = ParameterValue(args["serial_no"], value_type=str)
    # Direct parameters also cover dynamic SDK options (emitter_enabled), which
    # are not exposed as arguments by all versions of rs_launch.py.
    return [Node(package="realsense2_camera", executable="realsense2_camera_node",
                 namespace="camera", name="camera", parameters=[args], output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial", description="Actual D435 serial from SDK readback"),
        DeclareLaunchArgument("mode", default_value="stereo", choices=["stereo", "rgbd"]),
        DeclareLaunchArgument("width", default_value="640"),
        DeclareLaunchArgument("height", default_value="480"),
        DeclareLaunchArgument("fps", default_value="30"),
        OpaqueFunction(function=setup),
    ])
