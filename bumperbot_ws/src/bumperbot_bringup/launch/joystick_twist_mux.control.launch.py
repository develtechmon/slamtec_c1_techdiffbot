from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package = get_package_share_directory('bumperbot_bringup')

    joy_params_twist = os.path.join(get_package_share_directory('bumperbot_bringup'),'config','joystick_twist_mux.yaml')

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop",
        parameters=[joy_params_twist],
        #parameters=[os.path.join(get_package_share_directory("bumperbot_bringup"), "config", "joystick_twist_mux.yaml"),
        #            {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    joy_nodes = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[joy_params_twist],
        #parameters=[os.path.join(get_package_share_directory("bumperbot_bringup"), "config", "joystick_twist_mux.yaml"),
        #            {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    twist_mux_launch = IncludeLaunchDescription(
    os.path.join(
        get_package_share_directory("twist_mux"),
        "launch",
        "twist_mux_launch.py"
    ),
    launch_arguments={
        "cmd_vel_out": "/diff_cont/cmd_vel_unstamped",
        "config_locks": os.path.join(package, "config", "twist_mux_locks.yaml"),
        "config_topics": os.path.join(package, "config", "twist_mux_topics.yaml"),
        "config_joy": os.path.join(package, "config", "twist_mux_joy.yaml"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }.items(),
    )

    twist_relay_node = Node(
        package="bumperbot_bringup",
        executable="twist_relay",
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        joy_teleop,
        joy_nodes,
        twist_mux_launch,
        twist_relay_node,
    ])
