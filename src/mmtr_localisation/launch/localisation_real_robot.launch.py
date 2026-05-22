import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import TimerAction


def generate_launch_description():
    # cov_overide = Node(package="mmtr_localisation", executable="odom_covariance_override.py",    parameters=[{"use_sim_time": True}])
    # imu_calibration = Node(package="mmtr_calibration", executable="imu_calibration.py",    parameters=[{"use_sim_time": True}])
    # mag_frame_conversion = Node(package="mmtr_localisation", executable="magnetometer_frame_conversion.py", parameters=[{"use_sim_time":True}])
   

    madgwick_filter = Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="madgwick_filter",
            output="screen",
            parameters=[
                {
                    "use_sim_time":False,
                    "use_mag": False,
                    "world_frame": "enu",
                    "publish_tf": False,
                    "gain": 0.15,
                    "zeta": 0.08,
                }
            ],

        )
    
    zupt = Node(package="mmtr_localisation", executable="simple_zupt.py", parameters=[{"use_sim_time":False}])
    # zero_yaw = Node(package="mmtr_localisation", executable="zero_yaw.py", parameters=[{"use_sim_time":True}])
    
    robot_localization_madgwick_ukf = Node(
        package="robot_localization",
        executable="ukf_node",
        name="ukf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("mmtr_localisation"), "config", "ukf_real_robot.yaml"
            ),
            {"use_sim_time": False},
        ],
        remappings=[("/odometry/filtered", "/odometry/filtered_ukf")],
    )


    ld = LaunchDescription()
    # ld.add_action(cov_overide)
    # ld.add_action(imu_calibration)
    # ld.add_action(mag_frame_conversion)
    ld.add_action(madgwick_filter)
    ld.add_action(zupt)
    # ld.add_action(zero_yaw)
    ld.add_action(TimerAction(period=5.0, actions=[robot_localization_madgwick_ukf]))


    return ld
