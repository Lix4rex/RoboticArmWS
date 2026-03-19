import launch
from launch.substitutions import Command
import launch_ros
import os

packageName = "robotic_arm"

xacroRelativePath            = "model/model.real.xacro"
rvizRelativePath             = "config/config_real.rviz"
paramsConfigRelativePath = "config/params.yaml"
controllerConfigRelativePath = "config/controller.yaml"


def generate_launch_description():

    # ── Paths ────────────────────────────────────────────────────────────────
    pkgPath = launch_ros.substitutions.FindPackageShare(
        package=packageName
    ).find(packageName)

    xacroModelPath        = os.path.join(pkgPath, xacroRelativePath)
    rvizConfigPath        = os.path.join(pkgPath, rvizRelativePath)
    paramsConfigPath  = os.path.join(pkgPath, paramsConfigRelativePath)
    controllerConfigPath  = os.path.join(pkgPath, controllerConfigRelativePath)

    # ── Robot description ────────────────────────────────────────────────────
    robot_desc        = Command(['xacro ', xacroModelPath])
    robot_description = {"robot_description": robot_desc}

    # ── Robot State Publisher ────────────────────────────────────────────────
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {
            "use_sim_time": False,
            "publish_frequency": 50.0
        }]
    )

    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": Command(['xacro ', xacroModelPath])},
            controllerConfigPath
        ],
        output="screen"
    )
    
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    position_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller"],
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizConfigPath],
        parameters=[{"use_sim_time": False}]
    )

    # ── Controller (ton node custom) ─────────────────────────────────────────
    arm_controller = launch_ros.actions.Node(
        package="robotic_arm",
        executable="arm_controller",
        parameters=[paramsConfigPath]
    )

    # ── Liste finale ─────────────────────────────────────────────────────────
    return launch.LaunchDescription([
        control_node,
        robot_state_publisher_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        position_controller_spawner,
        arm_controller
    ])