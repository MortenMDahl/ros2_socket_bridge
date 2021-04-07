# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    mod_bringup_dir = get_package_share_directory('rdb_nav2')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace_1 = LaunchConfiguration('namespace_1')
    namespace_2 = LaunchConfiguration('namespace_2')
    namespace_3 = LaunchConfiguration('namespace_3')
    namespace_4 = LaunchConfiguration('namespace_4')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file_1 = LaunchConfiguration('params_file_1')
    params_file_2 = LaunchConfiguration('params_file_2')
    params_file_3 = LaunchConfiguration('params_file_3')
    params_file_4 = LaunchConfiguration('params_file_4')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                   ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_namespace_cmd_1 = DeclareLaunchArgument(
        'namespace_1',
        default_value='robot1',
        description='Top-level namespace')

    declare_namespace_cmd_2 = DeclareLaunchArgument(
        'namespace_2',
        default_value='robot2',
        description='Top-level namespace')

    declare_namespace_cmd_3 = DeclareLaunchArgument(
        'namespace_3',
        default_value='robot3',
        description='Top-level namespace')

    declare_namespace_cmd_4 = DeclareLaunchArgument(
        'namespace_4',
        default_value='robot4',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='True',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_1_cmd = DeclareLaunchArgument(
        'params_file_1',
        default_value=os.path.join(mod_bringup_dir, 'params', 'nav2_multirobot_params_1.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_params_file_2_cmd = DeclareLaunchArgument(
        'params_file_2',
        default_value=os.path.join(mod_bringup_dir, 'params', 'nav2_multirobot_params_2.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_params_file_3_cmd = DeclareLaunchArgument(
        'params_file_3',
        default_value=os.path.join(mod_bringup_dir, 'params', 'nav2_multirobot_params_3.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_params_file_4_cmd = DeclareLaunchArgument(
        'params_file_4',
        default_value=os.path.join(mod_bringup_dir, 'params', 'nav2_multirobot_params_4.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # TODO(orduno) Switch back once ROS argument passing has been fixed upstream
        #              https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/91
        # default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
        #                            'worlds/turtlebot3_worlds/waffle.model'),
        default_value=os.path.join(mod_bringup_dir, 'worlds', '4_tb3_world.model'),
        description='Full path to world model file to load')

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', world],
        cwd=[launch_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')

    urdf = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')

    start_robot_state_publisher_1_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_1',
        namespace=namespace_1,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        arguments=[urdf])

    start_robot_state_publisher_2_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_2',
        namespace=namespace_2,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        arguments=[urdf])

    start_robot_state_publisher_3_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_3',
        namespace=namespace_3,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        arguments=[urdf])

    start_robot_state_publisher_4_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_4',
        namespace=namespace_4,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        arguments=[urdf])

    rviz_cmd_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': 'robot1',
                          'use_namespace': 'True',
                          'rviz_config': rviz_config_file}.items())

    rviz_cmd_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': 'robot2',
                          'use_namespace': 'True',
                          'rviz_config': rviz_config_file}.items())

    rviz_cmd_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': 'robot3',
                          'use_namespace': 'True',
                          'rviz_config': rviz_config_file}.items())

    rviz_cmd_4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': 'robot4',
                          'use_namespace': 'True',
                          'rviz_config': rviz_config_file}.items())

    bringup_cmd_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace_1,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file_1,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())

    bringup_cmd_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace_2,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file_2,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())

    bringup_cmd_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace_3,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file_3,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())

    bringup_cmd_4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace_4,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file_4,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd_1)
    ld.add_action(declare_namespace_cmd_2)
    ld.add_action(declare_namespace_cmd_3)
    ld.add_action(declare_namespace_cmd_4)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_1_cmd)
    ld.add_action(declare_params_file_2_cmd)
    ld.add_action(declare_params_file_3_cmd)
    ld.add_action(declare_params_file_4_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_1_cmd)
    ld.add_action(start_robot_state_publisher_2_cmd)
    ld.add_action(start_robot_state_publisher_3_cmd)
    ld.add_action(start_robot_state_publisher_4_cmd)

    ld.add_action(rviz_cmd_1)
    ld.add_action(rviz_cmd_2)
    ld.add_action(rviz_cmd_3)
    ld.add_action(rviz_cmd_4)

    ld.add_action(bringup_cmd_1)
    ld.add_action(bringup_cmd_2)
    ld.add_action(bringup_cmd_3)
    ld.add_action(bringup_cmd_4)
    return ld