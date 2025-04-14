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
            "robot_description": Command(["xacro ", urdf_file])
        }]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui"
    )
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([pkg_path, "rviz", "display.rviz"])],
        output="screen"
    )
    
    return LaunchDescription([
        robot_state_publisher,
        # joint_state_publisher_gui,
        rviz2
    ])