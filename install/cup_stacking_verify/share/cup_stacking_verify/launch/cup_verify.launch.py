"""Bring up cup_stacking_verify, wired to the depth_digital_twin pipeline.

This is the vision-node side of the integration. depth_digital_twin runs in
its own terminals (robot / camera / digital_twin.launch.py / pick_ui_node) and
publishes per-cup 3D boxes on /digital_twin/boxes with its OWN RViz window.

This launch adds, in a SEPARATE RViz window:

  boxes_to_detections_node  /digital_twin/boxes (MarkerArray, world)
                              → /detected_cups (Detection3DArray)
  cup_occupancy_verifier    /detected_cups → /cup_occupancy_status,
                              /cup_overlap_ratio, /virtual_cup_markers
  topic_logger_node         prints the above topics as text
  rviz2                     vision-node window (cup_verify.rviz)

Run order (depth_digital_twin terminals 1-4 first), then:

  ros2 launch cup_stacking_verify cup_verify.launch.py

Args:
  rviz             : true|false                 (default: true)
  rviz_config      : path to .rviz              (default: package rviz/)
  boxes_topic      : depth_digital_twin boxes   (default: /digital_twin/boxes)
  detections_topic : bridged Detection3DArray   (default: /detected_cups)
  target_frame     : marker / detection frame   (default: world)
  threshold        : occupancy overlap threshold (default: 0.6)
  use_test_pub     : true → run synthetic test_pub instead of the bridge
                     (standalone testing without depth_digital_twin)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('cup_stacking_verify')

    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    boxes_topic = LaunchConfiguration('boxes_topic')
    detections_topic = LaunchConfiguration('detections_topic')
    target_frame = LaunchConfiguration('target_frame')
    threshold = LaunchConfiguration('threshold')
    use_test_pub = LaunchConfiguration('use_test_pub')

    bridge = Node(
        package='cup_stacking_verify', executable='boxes_to_detections',
        name='boxes_to_detections_node', output='screen',
        condition=UnlessCondition(use_test_pub),
        parameters=[{
            'boxes_topic': boxes_topic,
            'detections_topic': detections_topic,
            'target_frame': target_frame,
        }])

    test_pub = Node(
        package='cup_stacking_verify', executable='test_publisher',
        name='test_publisher', output='screen',
        condition=IfCondition(use_test_pub))

    verifier = Node(
        package='cup_stacking_verify', executable='verifier',
        name='cup_occupancy_verifier', output='screen',
        parameters=[{
            'target_frame': target_frame,
            'threshold': threshold,
        }])

    logger = Node(
        package='cup_stacking_verify', executable='topic_logger',
        name='topic_logger_node', output='screen',
        parameters=[{'detections_topic': detections_topic}])

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_cup_verify',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz), output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [pkg_share, 'rviz', 'cup_verify.rviz'])),
        DeclareLaunchArgument(
            'boxes_topic', default_value='/digital_twin/boxes'),
        DeclareLaunchArgument(
            'detections_topic', default_value='/detected_cups'),
        DeclareLaunchArgument('target_frame', default_value='world'),
        DeclareLaunchArgument('threshold', default_value='0.6'),
        DeclareLaunchArgument('use_test_pub', default_value='false'),
        bridge,
        test_pub,
        verifier,
        logger,
        rviz_node,
    ])
