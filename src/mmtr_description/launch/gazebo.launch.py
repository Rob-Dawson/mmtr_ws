import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mmtr_description_dir = get_package_share_directory("mmtr_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(mmtr_description_dir, "urdf", "mmtr.urdf.xacro"),
        description="Absolute path to the robot file",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    set_gazebo_model_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[FindPackageShare("mmtr_description"), "/meshes"],
    )

    default_world = os.path.join(mmtr_description_dir, "worlds", "empty.sdf")
    world = LaunchConfiguration("world")
    world_arg = DeclareLaunchArgument(
        "world", default_value=default_world, description="World to load"
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [
                "r -v4 ",
                world,
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/model/mmtr/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/model/mmtr/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "world/empty/model/wall/link/box/sensor/wall_contact_sensor/contact@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts",
            "world/empty/model/wall2/link/box/sensor/wall_contact_sensor/contact@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts",
            "world/empty/pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V",
            "/magnetometer@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer",
        ],
        remappings=[
            ("/imu", "imu/out"),
            (
                "/world/empty/model/wall/link/box/sensor/wall_contact_sensor/contact",
                "contact",
            ),
            (
                "/world/empty/model/wall2/link/box/sensor/wall_contact_sensor/contact",
                "contact",
            ),
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "mmtr",
            "-topic",
            "robot_description",
            "-z",
            "0.03494100027694504",
        ],
        output="screen",
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    ld = LaunchDescription()
    ld.add_action(model_arg)
    ld.add_action(world_arg)
    ld.add_action(set_gazebo_model_path)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_broadcaster_node)
    ld.add_action(gz_ros2_bridge)

    return ld
