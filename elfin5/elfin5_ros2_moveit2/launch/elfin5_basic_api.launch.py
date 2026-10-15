#!/usr/bin/python3

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:

        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():

    # 工位编号: 非空则用 cos_rsvisual 带 D435i 的 URDF, 相机按
    # config/hand_eye/station<N>.yaml 的手眼标定外参挂到 elfin_end_link;
    # 空字符串回退厂家无相机 URDF(排查对比用)。
    station_arg = DeclareLaunchArgument(
        "station", default_value="06",
        description="工位编号(cos_rsvisual/config/hand_eye/station<N>.yaml); 空=厂家无相机URDF")

    def _make_basic_api_node(context):
        station = LaunchConfiguration("station").perform(context)
        xacro_path = None
        mappings = {'use_fake_hardware': 'false', 'use_real_hardware': 'true'}
        if station:
            try:
                cos_share = get_package_share_directory("cos_rsvisual")
                with open(os.path.join(
                        cos_share, 'config', 'hand_eye', f'station{station}.yaml')) as f:
                    cfg = yaml.safe_load(f)
                xacro_path = os.path.join(
                    cos_share, 'urdf', 'elfin5_rs_d435i.urdf.xacro')
                mappings['mount_xyz'] = cfg['mount_xyz']
                mappings['mount_rpy'] = cfg['mount_rpy']
                print(f'[elfin5_basic_api] 工位 {station}: 带相机 URDF, '
                      f'mount_xyz="{cfg["mount_xyz"]}" mount_rpy="{cfg["mount_rpy"]}"')
            except Exception as e:
                print(f'[elfin5_basic_api] 警告: 工位 {station} 标定读取失败({e}), '
                      '回退厂家无相机 URDF')
        if xacro_path is None:
            xacro_path = os.path.join(
                get_package_share_directory("elfin5_ros2_gazebo"),
                "urdf", "elfin5.urdf.xacro")
            print('[elfin5_basic_api] 使用厂家 URDF(无相机)')
        robot_description_config = xacro.process_file(xacro_path, mappings=mappings)
        robot_description = {"robot_description": robot_description_config.toxml()}
        # Robot description, SRDF:
        robot_description_semantic_config = load_file(
            "elfin5_ros2_moveit2", "config/elfin5.srdf"
        )
        robot_description_semantic = {
            "robot_description_semantic": robot_description_semantic_config
        }

        # Kinematics.yaml file:
        kinematics_yaml = load_yaml(
            "elfin5_ros2_moveit2", "config/kinematics.yaml"
        )
        # MoveIt 从 robot_description_kinematics.<group> 读取 IK 插件配置
        robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

        return [Node(
            name="elfin_basic_node",
            package="elfin_basic_api",
            executable="elfin_basic_api_node",
            output="screen",
            parameters=[robot_description,robot_description_semantic,robot_description_kinematics,{'use_sim_time': True}],
        )]

    return LaunchDescription(
        [
            station_arg,
            OpaqueFunction(function=_make_basic_api_node),
        ]
    )