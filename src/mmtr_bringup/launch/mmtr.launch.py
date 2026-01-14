import os
import rclpy
from rclpy.node import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory

def generate_launch_description():
	gazebo = IncludeLaunchDescription(
		os.path.join(get_package_share_directory("mmtr_description"),
		"launch", "gazebo.launch.py"
		),
	)

	flipper_controller = IncludeLaunchDescription(
		os.path.join(get_package_share_directory("mmtr_flipper_controller"),
		"launch", "flipper_controller.launch.py"),
	)

	localisation = IncludeLaunchDescription(
		os.path.join(get_package_share_directory("mmtr_localisation"), 
		"launch", "localisation.launch.py"),
	)

	local_mapping = IncludeLaunchDescription(
		os.path.join(get_package_share_directory("mmtr_local_mapping"),
		"launch", "local_map.launch.py"),
	)
	
	collision_detection = IncludeLaunchDescription(
		os.path.join(get_package_share_directory("mmtr_collision_detection"),
		"launch", "collision_detection.launch.py"),
	)
	

	ld = LaunchDescription()
	ld.add_action(gazebo)
	ld.add_action(flipper_controller)
	ld.add_action(localisation)
	ld.add_action(local_mapping)
	ld.add_action(collision_detection)

	return ld
