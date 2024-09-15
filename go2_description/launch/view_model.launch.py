from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

  urdf_file = PathJoinSubstitution(
    [FindPackageShare('go2_description'), 'urdf', 'go2.urdf']
  )
  rviz_config_file = PathJoinSubstitution(
    [FindPackageShare('go2_description'), 'rviz', 'view_model.rviz']
  )

  joint_state_publisher_node = Node(
    name="joint_state_publisher_node", 
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui",
    output="screen",
    arguments=[urdf_file]
  )

  robot_state_publisher_node = Node(
    name="robot_state_publisher_node",
    package="robot_state_publisher", 
    executable="robot_state_publisher",
    output="screen",
    arguments=[urdf_file]
  )

  rviz_node = Node(
    package="rviz2", 
    executable="rviz2", 
    name="rviz2",
    output="screen",
    arguments=["-d", rviz_config_file],
  )

  node_to_start = [
    joint_state_publisher_node,
    robot_state_publisher_node, 
    rviz_node
  ]

  return LaunchDescription(node_to_start)