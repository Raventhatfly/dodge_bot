from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_path = FindPackageShare("dodge_bringup")
    urdf_file = PathJoinSubstitution([pkg_path, "urdf", "dodgebot.xacro"])
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_file]),
            'publish_frequency': 1000.0
        }]
    )
    
    irmv_detection = Node(
        package="irmv_detection",
        executable="irmv_detector",
        name="irmv_detector",
        output="both",
    )

    armor_tracker = Node(
        package="armor_tracker",
        executable="armor_tracker_node",
        name="armor_tracker",
        output="both"
    )

    pos_interpreter = Node(
        package="solais_interpreter",
        executable="solais_interpreter_node",
        name="pos_interpreter",
        output="both",
    )
    
    return LaunchDescription([
        robot_state_publisher,
        armor_tracker,
        irmv_detection,
        pos_interpreter
    ])