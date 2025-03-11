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
        name="GZ_SIM_RESOURCE_PATH", 
        value=[FindPackageShare("mmtr_description"), "/meshes"]
    )
    default_world=os.path.join(mmtr_description_dir, 'worlds', 'empty.sdf')
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument('world', default_value=default_world, description='World to load')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": ['r -v4 ', world], 'on_exit_shutdown':'true'}.items()
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/model/mmtr/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/model/mmtr/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/model/mmtr/tf@tf2_msgs/msg/TFMessage@gz.msgs/Pose_V",
        ],
        remappings=[
            ("/imu", "imu/out")
        ]
    )


    spawn_robot = Node(package="ros_gz_sim", executable="create",
                       arguments=[
                           "-name", "mmtr",
                           "-topic", "robot_description",
                           '-z', '0.006'

                       ],
                       output="screen")

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    flipper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "flipper_controller",
            "--controller-manager",
            "/controller_manager"

        ]
    )

    odom_tf = Node(
        package="mmtr_description",
        executable="gz_tf_bridge.py",
        output="screen"
    )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0","--z", "0.103",
                   "--qx", "1", "--qy", "0", "--qz", "0", "--qw", "0",
                   "--frame-id", "base_footprint_ekf",
                   "--child-frame-id", "imu_link_ekf"],
    )


    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("mmtr_description"), "config", "ekf.yaml")],
    )


    imu_republisher_cpp = Node(
        package="mmtr_description",
        executable="imu_republisher",
    )




    ld = LaunchDescription()
    ld.add_action(model_arg)
    ld.add_action(world_arg)
    ld.add_action(set_gazebo_model_path)

    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(joint_state_broadcaster_node)
    ld.add_action(flipper_controller)
    ld.add_action(gz_ros2_bridge)


    ld.add_action(odom_tf)
    ld.add_action(static_transform_publisher)
    ld.add_action(robot_localization)
    
    ld.add_action(imu_republisher_cpp)





    # ld.add_action(custom_pid_node)




    # ld.add_action(flipper_controller)
    return ld
