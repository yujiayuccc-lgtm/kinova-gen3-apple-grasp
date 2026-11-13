from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    upstream = PathJoinSubstitution([
        FindPackageShare("kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
        "launch",
        "robot.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        GroupAction([
            # This parameter is inherited by move_group. Even if a client were
            # accidentally to request execution, MoveIt rejects it.
            SetParameter(name="allow_trajectory_execution", value=False),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(upstream),
                launch_arguments={
                    "robot_ip": robot_ip,
                    "launch_rviz": launch_rviz,
                    "use_internal_bus_gripper_comm": "true",
                }.items(),
            ),
        ]),
    ])
