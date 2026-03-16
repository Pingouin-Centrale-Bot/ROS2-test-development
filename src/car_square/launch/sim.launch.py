import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

packageName = "car_square"

xacroRelativePath        = "model/model_simu.xacro"
rvizRelativePath         = "config/config.rviz"
ros2controlRelativePath  = "config/robot_controller.yaml"
worldRelativePath        = "config/world.sdf"
controllerRelativeParams = "config/params.yaml"


def generate_launch_description():

    # ── Paths ────────────────────────────────────────────────────────────────
    pkgPath           = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)
    xacroModelPath    = os.path.join(pkgPath, xacroRelativePath)
    rvizConfigPath    = os.path.join(pkgPath, rvizRelativePath)
    ros2controlPath   = os.path.join(pkgPath, ros2controlRelativePath)
    worldPath         = os.path.join(pkgPath, worldRelativePath)

    robot_desc        = Command(['xacro ', xacroModelPath])
    robot_description = {"robot_description": robot_desc}

    controller_config = os.path.join(pkgPath, controllerRelativeParams)

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
        launch_arguments=[("gz_args", [" -r -v 3 ", worldPath])],
        condition=launch.conditions.IfCondition(gui)
    )

    # ── Bridges ──────────────────────────────────────────────────────────────
    clock_bridge = launch_ros.actions.Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen"
    )

    lidar_bridge = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan']
    )

    # Gazebo publie TOUS les joints (hub + rollers) → /joint_states_gz
    gz_joint_state_bridge = launch_ros.actions.Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/empty/model/robot_system_position/joint_state"
            "@sensor_msgs/msg/JointState[gz.msgs.Model"
        ],
        remappings=[
            ("/world/simple_world/model/robot_system_position/joint_state", "/joint_states_gz")
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
            "-x", "0", "-y", "0", "-z", "0.2"
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

    # joint_state_broadcaster publie les 4 roues → /joint_states_ros2control
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": True}]
    )

    # joint_state_publisher fusionne les deux sources → /joint_states
    # (robot_state_publisher écoute /joint_states)
    joint_state_merger = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{
            "use_sim_time": True,
            "rate": 50.0
        }]
    )

    # ── Controllers ──────────────────────────────────────────────────────────
    robot_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_drive_controller", "--param-file", ros2controlPath],
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

    # ── Séquençage : controllers + merger démarrent après le spawn ───────────
    post_spawn = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
                robot_controller_spawner,
                joint_state_merger,
            ]
        )
    )

    car_controller = launch_ros.actions.Node(
        package="car_square",
        executable="car_controller",
        parameters = [controller_config]
    )

    joint_states_exporter = launch_ros.actions.Node(
        package="car_square",
        executable="joint_states_exporter"
    )

    joy_node = launch_ros.actions.Node(
        package="joy",
        executable="joy_node",
        parameters=[{
            "device_id": 0,
        }]
    )

    # ── Description finale ───────────────────────────────────────────────────
    node_list = [
        gazebo,
        clock_bridge,
        lidar_bridge,
        gz_joint_state_bridge,
        gz_spawn_entity,
        delayed_rsp,
        rviz_node,

        car_controller,
        joint_states_exporter,

        post_spawn
    ]

    return launch.LaunchDescription(declared_arguments + node_list)