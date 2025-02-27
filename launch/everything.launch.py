import os
from ament_index_python.packages import (get_package_prefix, get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo)
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import (Node, SetParameter)

def generate_launch_description():
    package_description = "robot_description"
    package_directory = get_package_share_directory(package_description)

    package_description2 = "rg2_description"
    package_directory2 = get_package_share_directory(package_description2)
    install_dir_path2 = (get_package_prefix(package_description2) + "/share")
    robot_meshes_path2 = os.path.join(package_directory2, "meshes")
    pkg_models_path2 = os.path.join(package_directory2, "models")

    install_dir_path = (get_package_prefix(package_description) + "/share")
    robot_meshes_path = os.path.join(package_directory, "meshes")
    pkg_models_path = os.path.join(package_directory, "models")
    gazebo_resource_paths = [install_dir_path, robot_meshes_path, pkg_models_path]
    gazebo_resource_paths = [install_dir_path, robot_meshes_path, pkg_models_path, install_dir_path2, robot_meshes_path2, pkg_models_path2]
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["IGN_GAZEBO_RESOURCE_PATH"]:
                os.environ["IGN_GAZEBO_RESOURCE_PATH"] += (':' + resource_path)
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (':'.join(gazebo_resource_paths))

    # Load Demo World SDF from Robot Description Package #
    world_file = "demo_world_actor.sdf"
    world_file_path = os.path.join(package_directory, "worlds", world_file)
    world_config = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument("world",
                                              default_value=["-r ", world_file_path],
                                              description="SDF World File")
    
    # Declare GazeboSim Launch #
    gzsim_pkg = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gzsim_pkg, "launch", "gz_sim.launch.py"])),
            launch_arguments={"gz_args": world_config}.items(),
    )

    declare_urdf_file = DeclareLaunchArgument(
        'urdf_file',
        default_value='robot.xacro',
        description='URDF file to be loaded'
    )
    declare_spawn_model_name = DeclareLaunchArgument("model_name", default_value="my_robot",
                                                    description="Model Spawn Name")
    declare_spawn_x = DeclareLaunchArgument("x", default_value="-2.0", description="Model Spawn X Axis Value")
    declare_spawn_y = DeclareLaunchArgument("y", default_value="0.0", description="Model Spawn Y Axis Value")
    declare_spawn_z = DeclareLaunchArgument("z", default_value="0.5", description="Model Spawn Z Axis Value")

    urdf_file = LaunchConfiguration('urdf_file')
    robot_desc_path = PathJoinSubstitution([package_directory, "urdf", urdf_file])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output="screen",
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="my_robot_spawn",
        arguments=[
            "-name", LaunchConfiguration("model_name"),
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )

    # ROS-Gazebo Bridge #
    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
            "/cmd_vel" + "@geometry_msgs/msg/Twist" + "@ignition.msgs.Twist",
            "/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
            "/odom" + "@nav_msgs/msg/Odometry" + "[ignition.msgs.Odometry",
            "/laser/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
            "/imu" + "@sensor_msgs/msg/Imu" + "[ignition.msgs.IMU",
            # camera
            "/camera@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
        ],
        remappings=[
            # there are no remappings for this robot description
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_world_arg,
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            declare_urdf_file,
            declare_spawn_model_name,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_z,
            LogInfo(msg=['Using URDF file: ', urdf_file]),
            robot_state_publisher_node,
            gz_spawn_entity,
            ign_bridge,
        ]
    )

#ros2 launch robot_description everything.launch.py urdf_file:=camera.urdf.xacro x:=5 y:=5 z:=0.5