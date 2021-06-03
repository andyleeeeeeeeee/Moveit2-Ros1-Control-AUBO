import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

use_sim_time = False

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    # planning_context
    robot_description_config = load_file('aubo_description', 'urdf/aubo_i5.urdf')
    robot_description = {'robot_description' : robot_description_config}

    robot_description_semantic_config = load_file('aubo_i5_moveit_config', 'config/aubo_i5.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml('aubo_i5_moveit_config', 'config/kinematics.yaml')

    # Planning Functionality
    ompl_planning_pipeline_config = { 'move_group' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 0.1 } }
    ompl_planning_yaml = load_yaml('aubo_i5_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    controllers_yaml = load_yaml('aubo_i5_moveit_config', 'config/controllers.yaml')
    moveit_controllers = { 'moveit_simple_controller_manager' : controllers_yaml,
                           'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    trajectory_execution = {'moveit_manage_controllers': True,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 20.0,
                            'trajectory_execution.allowed_start_tolerance': 0.1}

    planning_scene_monitor_parameters = {"publish_planning_scene": True,
                 "publish_geometry_updates": True,
                 "publish_state_updates": True,
                 "publish_transforms_updates": True}

    # Sensor Plugin
    perception_config = {'octomap_frame': 'camera_link',
                        'octomap_resolution': 0.01}
    sensor_yaml = load_yaml('aubo_i5_moveit_config', 'config/sensor.yaml')

    # Start the actual move_group node/action server
    move_group_node = Node(package='moveit_ros_move_group',
                               executable='move_group',
                               output='screen',
                               parameters=[robot_description,
                                           robot_description_semantic,
                                           kinematics_yaml,
                                           ompl_planning_pipeline_config,
                                           trajectory_execution,
                                           moveit_controllers,
                                           planning_scene_monitor_parameters,
                                           perception_config,
                                           sensor_yaml,                                           
                                           {"use_sim_time": use_sim_time},
                                           {'source_list': ["/tf", "/fake_controller_joint_states"]}])
    # # RViz
    rviz_config_file = get_package_share_directory('aubo_i5_moveit_config') + "/launch/moveit.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description,
                                 robot_description_semantic,
                                 ompl_planning_pipeline_config,
                                 kinematics_yaml, 
                                 {"use_sim_time":use_sim_time}])

    # Static TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='base_to_camera',
                     output='log',
                     arguments=['-0.109843','1.59267','2.42131','-0.00469696', '0.99714029', '-0.07541542', '-0.00135503', 'base_link', 'camera_link'])

    # Publish TF
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description,
                                            {"use_sim_time":use_sim_time}])

    # Fake joint driver
    fake_joint_driver_node=Node(package='fake_joint_driver',
                                  executable='fake_joint_driver_node',
                                  parameters=[{'controller_name': 'aubo_i5_controller'},
                                              os.path.join(get_package_share_directory(
                                                  "aubo_i5_moveit_config"), "config", "auboi5_controllers.yaml"),
                                              os.path.join(get_package_share_directory(
                                                  "aubo_i5_moveit_config"), "config", "start_positions.yaml"),
                                              robot_description,
                                              {"use_sim_time": use_sim_time}])

    return LaunchDescription([static_tf, move_group_node, rviz_node, robot_state_publisher, fake_joint_driver_node])
    

    


