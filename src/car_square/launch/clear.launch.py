import launch
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import launch_ros
import os

packageName             = "car_square"
xacroRelativePath       = "model/model.xacro"
ros2controlRelativePath = "config/robot_controller.yaml"

def generate_launch_description():

    # ── Paths ─────────────────────────────────────────────────────────────────
    pkgPath         = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)
    xacroModelPath  = os.path.join(pkgPath, xacroRelativePath)
    ros2controlPath = os.path.join(pkgPath, ros2controlRelativePath)

    # ── Fix : ParameterValue force le type string pour xacro ─────────────────
    robot_desc        = ParameterValue(Command(['xacro ', xacroModelPath]), value_type=str)
    robot_description = {"robot_description": robot_desc}

    # ── ros2_control_node ─────────────────────────────────────────────────────
    ros2_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2controlPath],
        output="screen"
    )

    # ── robot_state_publisher ─────────────────────────────────────────────────
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"publish_frequency": 50.0}]
    )

    # ── Spawners (démarrent 2s après ros2_control_node) ───────────────────────
    robot_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_drive_controller", "--param-file", ros2controlPath],
    )

    post_start = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                launch.actions.TimerAction(period=2.0, actions=[
                    robot_controller_spawner,
                ])
            ]
        )
    )

    car_controller = launch_ros.actions.Node(
        package="car_square",
        executable="car_controller"
    )

    joy_node = launch_ros.actions.Node(
        package="joy",
        executable="joy_node",
        parameters=[{
            "device_id": 0,
        }]
    )

    return launch.LaunchDescription([
        ros2_control_node,
        robot_state_publisher_node,

        car_controller,

        post_start,
    ])