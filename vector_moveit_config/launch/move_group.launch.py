import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def load_yaml(package_path, file_path):
    full_path = os.path.join(package_path, file_path)
    with open(full_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    description_pkg = get_package_share_directory("vector_description")
    moveit_pkg = get_package_share_directory("vector_moveit_config")

    # --- Robot Description (URDF) ---
    xacro_file = os.path.join(description_pkg, "urdf", "vector_arm.urdf.xacro")
    robot_description = {"robot_description": xacro.process_file(xacro_file).toxml()}

    # --- SRDF ---
    srdf_file = os.path.join(moveit_pkg, "config", "vector_arm.srdf")
    with open(srdf_file, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # --- Kinematics ---
    kinematics_yaml = load_yaml(moveit_pkg, "config/kinematics.yaml")
    robot_description_kinematics = {
        "robot_description_kinematics": kinematics_yaml
    }

    # --- Joint Limits ---
    joint_limits_yaml = load_yaml(moveit_pkg, "config/joint_limits.yaml")
    robot_description_planning = {
        "robot_description_planning": joint_limits_yaml
    }

    # --- MoveIt Controllers ---
    moveit_controllers = load_yaml(moveit_pkg, "config/moveit_controllers.yaml")

    # --- Planning Pipeline (OMPL) ---
    planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
            ],
        },
    }

    # --- move_group Node ---
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            moveit_controllers,
            planning_pipelines,
            {"use_sim_time": True},
        ],
    )

    # --- RViz ---
    rviz_config = os.path.join(moveit_pkg, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])
