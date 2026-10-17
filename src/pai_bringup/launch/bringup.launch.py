"""Bring up the whole Physical AI workcell in one command.

Gazebo + Panda + ros2_control + RGB-D camera + MoveIt 2, and the pipeline on top: skills,
perception, world model, safety supervisor (+ simulated co-worker), task executor, operator
console (http://localhost:8080) and the rule-based baseline cell used by the benchmark.

    ros2 launch pai_bringup bringup.launch.py                       # Gazebo GUI + RViz
    ros2 launch pai_bringup bringup.launch.py headless:=true rviz:=false
    ros2 launch pai_bringup bringup.launch.py planner:=rule perception_backend:=claude
    ros2 launch pai_bringup bringup.launch.py scene:=/path/to/scene.yaml

The world SDF is generated from the scene YAML at launch time, so a new layout
needs no code changes.
"""
import importlib.util
import math
import os

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

BRINGUP = get_package_share_directory('pai_bringup')


def generate_world(scene_file):
    """Run make_world.py on the scene and return (world_path, scene_dict)."""
    script = os.path.join(get_package_prefix('pai_bringup'), 'lib', 'pai_bringup', 'make_world.py')
    spec = importlib.util.spec_from_file_location('make_world', script)
    make_world = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(make_world)
    with open(scene_file) as f:
        scene = yaml.safe_load(f)
    out_dir = os.path.join(os.environ.get('ROS_HOME', os.path.expanduser('~/.ros')), 'pai')
    os.makedirs(out_dir, exist_ok=True)
    world = os.path.join(out_dir, 'workcell.sdf')
    with open(world, 'w') as f:
        f.write(make_world.world_sdf(scene))
    return world, scene, make_world.graspable_names(scene)


def launch_setup(context):
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    use_moveit = LaunchConfiguration('moveit').perform(context) == 'true'
    world, scene, grasp_objects = generate_world(LaunchConfiguration('scene').perform(context))
    sim_time = {'use_sim_time': True}

    moveit_config = (
        MoveItConfigsBuilder('panda', package_name='pai_moveit_config')
        .robot_description(
            file_path=os.path.join(get_package_share_directory('pai_description'), 'urdf', 'panda_gz.urdf.xacro'),
            mappings={'controllers_file': os.path.join(BRINGUP, 'config', 'ros2_controllers.yaml'),
                      'grasp_objects': ' '.join(grasp_objects)})
        .robot_description_semantic(file_path='config/panda.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl', 'pilz_industrial_motion_planner'], default_planning_pipeline='ompl')
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs())

    gui = '' if LaunchConfiguration('gui_panels').perform(context) == 'true' else \
        f"--gui-config {os.path.join(BRINGUP, 'config', 'gazebo_gui.config')} "
    gz_args = f'-r -v 2 {gui}{world}'
    if headless:
        gz_args = f'-r -s --headless-rendering -v 2 {world}'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='log',
        parameters=[moveit_config.robot_description, sim_time])

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'panda', '-allow_renaming', 'false'])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '60'])

    jsb, arm, hand = spawner('joint_state_broadcaster'), spawner('panda_arm_controller'), spawner('panda_hand_controller')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='log',
        parameters=[sim_time],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # Ground-truth poses of movable models: used ONLY by the sim grasp and benchmark scoring.
            '/world/workcell/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/world/workcell/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ] + [f'/pai/grasp/{o}/{verb}@std_msgs/msg/Empty]ignition.msgs.Empty'
             for o in grasp_objects for verb in ('attach', 'detach')])

    # Camera extrinsics come from the same scene YAML that placed the camera in Gazebo.
    x, y, z, roll, pitch, yaw = scene['camera']['pose']
    camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', output='log',
        parameters=[sim_time],
        arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                   '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                   '--frame-id', 'world', '--child-frame-id', 'camera_link'])
    # Gazebo camera looks along +x; ROS optical frames look along +z (x right, y down).
    optical_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', output='log',
        parameters=[sim_time],
        arguments=['--roll', str(-math.pi / 2), '--yaw', str(-math.pi / 2),
                   '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame'])

    actions = [
        gazebo, robot_state_publisher, spawn_robot, bridge, camera_tf, optical_tf,
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, hand])),
    ]

    if LaunchConfiguration('skills').perform(context) == 'true':
        actions.append(Node(
            package='pai_skills', executable='skill_server', output='screen',
            parameters=[sim_time, {'grasp_objects': grasp_objects}]))

    if LaunchConfiguration('perception').perform(context) == 'true':
        actions.append(Node(
            package='pai_perception', executable='perception_node', output='screen',
            parameters=[sim_time, {'backend': LaunchConfiguration('perception_backend').perform(context)}]))
        actions.append(Node(package='pai_world_model', executable='world_model', output='screen',
                            parameters=[sim_time]))

    if LaunchConfiguration('safety').perform(context) == 'true':
        actions.append(Node(package='pai_safety', executable='supervisor', output='screen', parameters=[sim_time]))
        human = scene.get('human', {'name': 'worker', 'park': [2.2, -1.6]})
        actions.append(Node(package='pai_safety', executable='human_sim', output='screen',
                            parameters=[sim_time, {'name': human['name'], 'park': [float(v) for v in human['park']]}]))

    if LaunchConfiguration('executor').perform(context) == 'true':
        actions.append(Node(
            package='pai_planner', executable='executor', output='screen',
            parameters=[sim_time, {'planner': LaunchConfiguration('planner').perform(context)}]))

    if LaunchConfiguration('ui').perform(context) == 'true':
        actions.append(Node(package='pai_ui', executable='console', output='screen', parameters=[sim_time]))
    if LaunchConfiguration('baseline').perform(context) == 'true':
        actions.append(Node(package='pai_benchmark', executable='baseline', output='screen', parameters=[sim_time]))

    if use_moveit:
        actions.append(Node(
            package='moveit_ros_move_group', executable='move_group', output='log',
            parameters=[moveit_config.to_dict(), sim_time]))
        actions.append(Node(
            package='rviz2', executable='rviz2', output='log',
            arguments=['-d', os.path.join(BRINGUP, 'rviz', 'workcell.rviz')],
            parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics, moveit_config.planning_pipelines,
                        moveit_config.joint_limits, sim_time],
            condition=IfCondition(LaunchConfiguration('rviz'))))
    return actions


def generate_launch_description():
    # Gazebo resolves package:// mesh URIs through this path.
    share_roots = {os.path.dirname(get_package_share_directory(p))
                   for p in ('moveit_resources_panda_description', 'pai_description')}
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo without GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz with MoveIt panel'),
        DeclareLaunchArgument('gui_panels', default_value='false',
                              description='Show the Gazebo side panels (World inspector, Entity Tree)'),
        DeclareLaunchArgument('moveit', default_value='true', description='Start move_group'),
        DeclareLaunchArgument('skills', default_value='true', description='Start the skill action servers'),
        DeclareLaunchArgument('perception', default_value='true', description='Start perception + world model'),
        DeclareLaunchArgument('safety', default_value='true', description='Start safety supervisor + human simulator'),
        DeclareLaunchArgument('executor', default_value='true', description='Start the task executor'),
        DeclareLaunchArgument('planner', default_value='auto', description='auto | claude | rule'),
        DeclareLaunchArgument('ui', default_value='true', description='Operator console on http://localhost:8080'),
        DeclareLaunchArgument('baseline', default_value='true', description='Rule-based baseline cell (benchmark)'),
        DeclareLaunchArgument('perception_backend', default_value='geometric',
                              description='geometric | claude (Claude vision names the segments)'),
        DeclareLaunchArgument('scene', default_value=os.path.join(BRINGUP, 'config', 'scene_default.yaml'),
                              description='Scene YAML used to generate the world'),
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.pathsep.join(sorted(share_roots))),
        OpaqueFunction(function=launch_setup),
    ])
