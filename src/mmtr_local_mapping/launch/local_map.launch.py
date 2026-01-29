import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import TimerAction


def generate_launch_description():
    mmtr_local_map_dir = get_package_share_directory("mmtr_local_mapping")
    # local_map = Node(package="mmtr_local_mapping", executable="mapping_with_collisions.py",    parameters=[{"use_sim_time": True}])
    rviz2 = Node(package="rviz2", executable="rviz2",arguments=["-d", os.path.join(mmtr_local_map_dir, "config", "display.rviz")], parameters=[{}])

    local_mapping_costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="local_costmap",
        output="screen",
        parameters=[            
            os.path.join(
                get_package_share_directory("mmtr_local_mapping"), "config", "cost_map_static.yaml"
            ),]
    )

    local_mapping_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_local",
        output="screen",
        parameters=[{
            "autostart": True,
            "bond_timeout": 20.0,                 # <-- was 4.0
            "bond_heartbeat_period": 0.2,
            "node_names": ["local_costmap"],
            }],
    )

        

    ld = LaunchDescription()
    # ld.add_action(local_map)
    ld.add_action(rviz2)
    ld.add_action(local_mapping_costmap)
    ld.add_action(TimerAction(period=3.0, actions=[local_mapping_lifecycle]))    



    return ld
