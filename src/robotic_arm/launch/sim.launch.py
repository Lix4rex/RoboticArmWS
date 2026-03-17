import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

packageName = "robotic_arm"

xacroRelativePath        = "model/model.xacro"
rvizRelativePath         = "config/config.rviz"


def generate_launch_description():

    # ── Paths ────────────────────────────────────────────────────────────────
    pkgPath           = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)
    xacroModelPath    = os.path.join(pkgPath, xacroRelativePath)
    rvizConfigPath    = os.path.join(pkgPath, rvizRelativePath)

    robot_desc        = Command(['xacro ', xacroModelPath])
    robot_description = {"robot_description": robot_desc}

    # ── Arguments ────────────────────────────────────────────────────────────
    declared_arguments = [
        launch.actions.DeclareLaunchArgument(
            name="gui",
            default_value="true",
            description="Start Gazebo with GUI"
        )
    ]
    gui = LaunchConfiguration("gui")

    # ── Gazebo ───────────────────────────────────────────────────────────────
    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", [" -r -v 3 empty.sdf"])],
        condition=launch.conditions.IfCondition(gui)
    )

    # ── Bridges ──────────────────────────────────────────────────────────────
    clock_bridge = launch_ros.actions.Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen"
    )

    # Gazebo publie TOUS les joints (hub + rollers) → /joint_states_gz
    gz_joint_state_bridge = launch_ros.actions.Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/empty/model/robot_system_position/joint_state"
            "@sensor_msgs/msg/JointState[gz.msgs.Model"
        ],
        output="screen"
    )

    # ── Spawn ────────────────────────────────────────────────────────────────
    gz_spawn_entity = launch_ros.actions.Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name",  "robot_system_position",
            "-allow_renaming", "true",
            "-x", "0", "-y", "0", "-z", "0.0"
        ]
    )

    # ── State publishers ─────────────────────────────────────────────────────
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {
            "use_sim_time": True,
            "publish_frequency": 50.0
        }]
    )

    delayed_rsp = launch.actions.TimerAction(
        period=2.0,
        actions=[robot_state_publisher_node]
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": True}]
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizConfigPath],
        parameters=[{"use_sim_time": True}] 
    )

    # Spawner du controller
    position_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller"],
        parameters=[{"use_sim_time": True}]
    )

    # ── Séquençage : controllers + merger démarrent après le spawn ───────────
    post_spawn = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
                position_controller_spawner
            ]
        )
    )

    arm_controller = launch_ros.actions.Node(
        package="robotic_arm",
        executable="arm_controller"
    )

    # ── Description finale ───────────────────────────────────────────────────
    node_list = [
        gazebo,
        clock_bridge,
        gz_joint_state_bridge,
        gz_spawn_entity,
        delayed_rsp,
        rviz_node,
        post_spawn,

        arm_controller
    ]

    return launch.LaunchDescription(declared_arguments + node_list)