"""Launch the real visual ORB node; camera launched separately or replayed."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("stereo", "rgbd"):
        raise ValueError("mode must be stereo or rgbd")
    sim = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    return [Node(
        package="wbr_orb_node", executable="d435_node", name="d435_orb", output="screen",
        parameters=[{
            "mode": mode, "use_sim_time": sim,
            "orb_vocabulary": LaunchConfiguration("vocabulary").perform(context),
            "orb_settings": LaunchConfiguration("settings").perform(context),
            "max_skew_ns": 0 if mode == "stereo" else 10000000,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="stereo", choices=["stereo", "rgbd"]),
        DeclareLaunchArgument("vocabulary", description="Absolute path to ORBvoc.txt"),
        DeclareLaunchArgument("settings", description="SDK converter YAML for this device/profile/mode"),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=setup),
    ])
