from ament_index_python import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
import os
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


def generate_launch_description():

    mmtr_description_dir = get_package_share_directory("mmtr_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
        mmtr_description_dir, "urdf", "mmtr.urdf.xacro"),
        description="Absolute path to the robot file")

    world_name_arg = DeclareLaunchArgument(
        name="world_name", default_value="empty")

    world_file = PathJoinSubstitution([
        mmtr_description_dir, "worlds", "empty.world"
    ])

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    set_gazebo_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH", [FindPackageShare("mmtr_description"), "/meshes"]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"
            ])
        ),
        launch_arguments={
            "world":world_file
        }.items()
    )

    spawn_robot = Node(package="gazebo_ros", executable="spawn_entity.py",
                       arguments=[
                           "-entity", "mmtr",
                           "-topic", "robot_description",
                       ],
                       output="screen")

    ld = LaunchDescription()
    ld.add_action(model_arg)
    ld.add_action(world_name_arg)
    ld.add_action(set_gazebo_model_path)

    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    return ld
