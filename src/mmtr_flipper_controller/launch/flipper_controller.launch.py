from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    flipper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["flipper_controller", "--controller-manager", "/controller_manager"],
    )
    flipper_default_pose = Node(
        package="mmtr_flipper_controller",
        executable="flipper_default_pos.py",
    )

    ld = LaunchDescription()
    ld.add_action(joint_state_broadcaster_node)
    ld.add_action(flipper_controller)
    ld.add_action(flipper_default_pose)

    return ld
