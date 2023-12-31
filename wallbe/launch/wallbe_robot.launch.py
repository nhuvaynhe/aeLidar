import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node



def generate_launch_description():
    package_name='wallbe' #<--- CHANGE ME
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('wallbe'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')
    
    ekf_config = os.path.join(pkg_path,'config','ekf.yaml')
    slam_config = os.path.join(pkg_path,'config','mapper_params_online_async.yaml.yaml')

    rviz_config_file = os.path.join(pkg_path,'rviz','wallbe.rviz')

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}]
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_robot_localization_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_localization_node,
            on_start=[rviz_node],
        )
    )

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_config,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_slam_after_robot_localization = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_localization_node,
            on_start=[start_async_slam_toolbox_node],
        )
    )

    # Launch them all!
    return LaunchDescription([
        # node_robot_state_publisher,
        robot_localization_node,
        # diff_drive_spawner,
        # joint_broad_spawner,
        # delayed_controller_manager,
        delay_rviz_after_robot_localization_node,
        delay_slam_after_robot_localization
    ])
