from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    ##A full collision detection package might not be necessary, but i wanted it seperate for now
    # Especially if i do things like add in the collision rewind (track the odom for 2 seconds then
    # when a crash is detected, back track through that 2 seconds to find the clossest time which matches the
    # collision and then use the closest odom as a correction step)

    collision_detection_2D = Node(
        package="mmtr_collision_detection",
        executable="collision_detection_2D.py",
        parameters=[{"use_sim_time": True}],
    )
    jerk_calculation = Node(
        package="mmtr_collision_detection",
        executable="jerk_calc.py",
        parameters=[{"use_sim_time": True}]
    )
    collision_detection_3D = Node(
        package="mmtr_collision_detection",
        executable="collision_detection_3D.py",
        parameters=[{"use_sim_time": True}],
    )
    collision_detection_shim = Node(
        package="mmtr_collision_detection",
        executable="collision_detection_shim.py",
        parameters=[{"use_sim_time": True}],
    )

    ld = LaunchDescription()
    ld.add_action(jerk_calculation)
    ld.add_action(collision_detection_2D)

    return ld
