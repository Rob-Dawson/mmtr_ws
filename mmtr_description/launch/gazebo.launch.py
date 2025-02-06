from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import os
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
def generate_launch_description():

    mmtr_description_dir = get_package_share_directory("mmtr_description")
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
        mmtr_description_dir, "urdf", "mmtr.urdf.xacro"),
        description="Absolute path to the robot file")
    robot_description= ParameterValue(Command(["xacro", LaunchConfiguration("model")]),
                                      value_type=str)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    ld = LaunchDescription()
    ld.add_action(model_arg)
    ld.add_action(robot_state_publisher)
    return ld