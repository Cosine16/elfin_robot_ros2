#!/usr/bin/python3

# elfin5_moveit.launch.py:
# Launch file for the elfin5 Robot MoveIt!2 SIMULATION in ROS2 Humble:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,Command, FindExecutable, PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument,Shutdown,OpaqueFunction
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

    rviz_arg = DeclareLaunchArgument("rviz_file", default_value="False", description="Load RVIZ file.")
    # 工位编号: 非空则用 cos_rsvisual 带 D435i 的 URDF, 相机按
    # config/hand_eye/station<N>.yaml 的手眼标定外参挂到 elfin_end_link;
    # 空字符串回退厂家无相机 URDF(排查对比用)。
    station_arg = DeclareLaunchArgument(
        "station", default_value="06",
        description="工位编号(cos_rsvisual/config/hand_eye/station<N>.yaml); 空=厂家无相机URDF")

    def _make_robot_description(context):
        station = LaunchConfiguration("station").perform(context)
        xacro_args = [' use_fake_hardware:=false', ' use_real_hardware:=true']
        xacro_path = None
        if station:
            try:
                cos_share = get_package_share_directory("cos_rsvisual")
                with open(os.path.join(
                        cos_share, 'config', 'hand_eye', f'station{station}.yaml')) as f:
                    cfg = yaml.safe_load(f)
                xacro_path = os.path.join(
                    cos_share, 'urdf', 'elfin5_rs_d435i.urdf.xacro')
                xacro_args.append(f' mount_xyz:="{cfg["mount_xyz"]}"')
                xacro_args.append(f' mount_rpy:="{cfg["mount_rpy"]}"')
                print(f'[elfin5_moveit] 工位 {station}: 带相机 URDF, '
                      f'mount_xyz="{cfg["mount_xyz"]}" mount_rpy="{cfg["mount_rpy"]}"')
            except Exception as e:
                print(f'[elfin5_moveit] 警告: 工位 {station} 标定读取失败({e}), '
                      '回退厂家无相机 URDF')
        if xacro_path is None:
            xacro_path = os.path.join(
                get_package_share_directory("elfin5_ros2_gazebo"),
                "urdf", "elfin5.urdf.xacro")
            print('[elfin5_moveit] 使用厂家 URDF(无相机)')
        return {'robot_description': Command(
            [FindExecutable(name='xacro'), ' ', xacro_path] + xacro_args)}

    # 本文件只负责硬件侧: RSP + ros2_control_node。
    # (move_group/RViz 在 elfin5_moveit_rviz.launch.py; map 里曾经的死代码已删除)

    ros2_controllers_path = os.path.join(
        get_package_share_directory("elfin_robot_bringup"),
        "config",
        "elfin_arm_control.yaml",
    )

    def _make_robot_nodes(context):
        robot_description = _make_robot_description(context)

        #publish tf
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        )

        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description,ros2_controllers_path],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
        )
        return [robot_state_publisher, ros2_control_node]

    # static tf
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "elfin_base_link"],

    )

    return LaunchDescription(
        [
            station_arg,
            static_tf,
            OpaqueFunction(function=_make_robot_nodes),
        ]
    )
