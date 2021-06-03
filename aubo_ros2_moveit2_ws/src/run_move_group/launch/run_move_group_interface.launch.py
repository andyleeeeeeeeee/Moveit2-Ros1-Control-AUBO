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
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # planning_context
    robot_description_config = load_file('aubo_description', 'urdf/aubo_i5.urdf')
    robot_description = {'robot_description' : robot_description_config}

    robot_description_semantic_config = load_file('aubo_i5_moveit_config', 'config/aubo_i5.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml('aubo_i5_moveit_config', 'config/kinematics.yaml')

    # MoveGroupInterface demo executable
    run_move_group_demo = Node(name='run_move_group',
                               package='run_move_group',
                               executable='run_move_group',
                               prefix='xterm -e',
                               output='screen',
                               parameters=[robot_description,
                                           robot_description_semantic,
                                           kinematics_yaml,
                                           {"use_sim_time": use_sim_time}])

    return LaunchDescription([run_move_group_demo])
