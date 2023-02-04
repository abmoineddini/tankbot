import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    package_name = 'tankbot'
    urdf_file_path = 'description/tankbot_robot.urdf'
    controller_path = 'config/tankbot_controller.yaml'

    # Set the path to different files and folders.
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    controller_params_file = os.path.join(pkg_share, controller_path)

    # Launch configuration variables specific to simulation
    gui = LaunchConfiguration('gui')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_namespace = LaunchConfiguration('use_namespace')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    
    # Declare the launch arguments  
    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Flag to enable joint_state_publisher_gui')
        
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=default_urdf_model_path, 
        description='Absolute path to robot urdf file')
        
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
    start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': Command(['xacro ', urdf_model])}])

    # Publish the joint states of the robot
    start_joint_state_publisher_cmd = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    condition=UnlessCondition(gui))

    # ros2 control argument diff drive
    controller_manager_cmd = Node(
    package='controller_manager', 
    executable='ros2_control_node',
    parameters=[{'robot_description':Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])}, 
                controller_params_file])
    delayed_controller_manager_cmd = TimerAction(period=3.0, actions=[controller_manager_cmd])

    # ros2 control argument diff drive
    spawn_diff_cont_cmd = Node(
    package='controller_manager', 
    executable='spawner.py',
    arguments=["diff_cont"])
    delayed_spawn_diff_cont_cmd = RegisterEventHandler(event_handler=OnProcessStart(
                                                        target_action=controller_manager_cmd, 
                                                        on_start=spawn_diff_cont_cmd))

    # ros2 control argument joint broad
    spawn_joint_broad_cmd = Node(
    package='controller_manager', 
    executable='spawner.py',
    arguments=["joint_broad"])
    delayed_spawn_joint_broad_cmd = RegisterEventHandler(event_handler=OnProcessStart(
                                                        target_action=controller_manager_cmd, 
                                                        on_start=spawn_joint_broad_cmd))

    # ros2 lidar start
    start_lidar_cmd = Node(
    package='rplidar_ros', 
    executable='rplidar_composition')

    # Start Camera
    start_camera_cmd = Node(
    package='v4l2_camera', 
    executable='v4l2_camera_node',
    output = 'screen',
    parameters=[{
        'image_size' : [640, 480],
        'camera_frame_id' : 'camera_link_optical'
    }])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)  
    
    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(delayed_controller_manager_cmd)
    ld.add_action(delayed_spawn_diff_cont_cmd)
    ld.add_action(delayed_spawn_joint_broad_cmd)
    ld.add_action(start_lidar_cmd)
    ld.add_action(start_camera_cmd)

    return ld

# mapped Teleop command
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
# ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]" -p camera_frame_id:=camera_link_optical
# ros2 launch slam_toolbox online_async_launch.py params_file:=./src/tankbot/config/mapper_params_online_async.yaml use_sim_time:=false
