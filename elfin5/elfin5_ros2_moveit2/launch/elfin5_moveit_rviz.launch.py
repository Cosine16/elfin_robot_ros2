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
                print(f'[elfin5_moveit_rviz] 工位 {station}: 带相机 URDF, '
                      f'mount_xyz="{cfg["mount_xyz"]}" mount_rpy="{cfg["mount_rpy"]}"')
            except Exception as e:
                print(f'[elfin5_moveit_rviz] 警告: 工位 {station} 标定读取失败({e}), '
                      '回退厂家无相机 URDF')
        if xacro_path is None:
            xacro_path = os.path.join(
                get_package_share_directory("elfin5_ros2_gazebo"),
                "urdf", "elfin5.urdf.xacro")
            print('[elfin5_moveit_rviz] 使用厂家 URDF(无相机)')
        return {'robot_description': Command(
            [FindExecutable(name='xacro'), ' ', xacro_path] + xacro_args)}

    # load robot description, srdf
    robot_description_semantic_config = load_file(
        "elfin5_ros2_moveit2", "config/elfin5.srdf"
    )

    robot_description_semantic = {
        "robot_description_semantic":robot_description_semantic_config
    }

    # load kinematics.yaml
    kinematics_yaml = load_yaml(
        "elfin5_ros2_moveit2", "config/kinematics.yaml"
    )

    robot_description_kinematics = {"robot_description_kinematics":kinematics_yaml}

    # planning functionality
    ompl_planning_pipeline_config = {
        "move_group":{
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
        "start_state_max_bounds_error":0.1,
        }
    }

    ompl_planning_yaml = load_yaml(
        "elfin5_ros2_moveit2","config/ompl_planning.yaml"
    )

    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # trajectory execution functionality
    moveit_simple_controllers_yaml = load_yaml(
        "elfin5_ros2_moveit2", "config/elfin_controllers.yaml"
    )

    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates":  True,
        "publish_transforms_updates": True,
    }

    # OctoMap 点云避障: 注入 cos_rsvisual/config/sensors_3d.yaml
    # (PointCloudOctomapUpdater, 需 ros-humble-moveit-ros-perception)。
    # 点云话题本身由 cos_rsvisual 的相机 launch 开启(face_real 的
    # enable_pointcloud:=true); 相机没开时 updater 空订阅, 规划不受影响。
    sensors_3d_yaml = load_yaml(
        "cos_rsvisual", "config/sensors_3d.yaml"
    )

    # start move_group & action server + RViz (robot_description 依赖 station 参数,
    # 需在 OpaqueFunction 里等 launch context 再构造)
    def _make_moveit_nodes(context):
        robot_description = _make_robot_description(context)

        run_move_group_node = Node(
            package = "moveit_ros_move_group",
            executable = "move_group",
            output = "screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                ompl_planning_pipeline_config,
                trajectory_execution,
                moveit_controllers,
                planning_scene_monitor_parameters,
            ] + ([sensors_3d_yaml] if sensors_3d_yaml else []),
        )

        # Rviz
        load_rviz = LaunchConfiguration("rviz_file")
        rviz_base = os.path.join(get_package_share_directory("elfin5_ros2_moveit2"),"launch")
        rviz_full_config = os.path.join(rviz_base,"elfin5_moveit2.rviz")
        rviz_node_full = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_full_config],
            parameters=[
                robot_description,
                robot_description_semantic,
                ompl_planning_pipeline_config,
                robot_description_kinematics,
            ],
            condition=UnlessCondition(load_rviz),
        )
        return [run_move_group_node, rviz_node_full]

    # 注意: 本文件不再包含 static_tf / robot_state_publisher / ros2_control_node
    # (那些是 elfin5_moveit.launch.py 的职责, 此前这里的同名定义从未被启动, 已删除)。

    elfin_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["elfin_arm_controller", "--controller-manager", "/controller_manager"],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        [
            rviz_arg,
            station_arg,
            OpaqueFunction(function=_make_moveit_nodes),
            elfin_controller_spawner,
            joint_state_spawner
        ]
    )
