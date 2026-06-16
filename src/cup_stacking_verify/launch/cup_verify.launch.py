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
  pose_tuner_node           tkinter UI: live-edit p_start / v_dir
  rviz2                     vision-node window (cup_verify.rviz)

Geometry sync (verifier slots ↔ FastAPI pyramid placement)
----------------------------------------------------------
The verifier judges a slot occupied by overlapping detected cups against
virtual slot boxes anchored at `cp` (= L1_M centre) with row rotation `degree`.
Those MUST match where the robot actually places cups, which the FastAPI server
owns (GET /api/robot/config/pyramid → center{x,y} + degree). If they drift the
placed cup falls outside the slot box, /stack never flips to occupied, and the
LLM loop stalls waiting for the world update.

So the verifier node POLLS that endpoint at runtime (sync_pyramid_geometry,
pyramid_config_url, cp_z passed below) and mirrors center/degree into its own
cp/degree. Runtime polling — not a launch-time fetch — so it is independent of
process start order and self-heals once the FastAPI server (Docker) is up.
Set sync_pyramid_geometry:=false to pin cp/degree to params instead.

Run order (depth_digital_twin terminals 1-4 first), then:

  ros2 launch cup_stacking_verify cup_verify.launch.py

Args:
  rviz             : true|false                 (default: true)
  rviz_config      : path to .rviz              (default: package rviz/)
  boxes_topic      : depth_digital_twin boxes   (default: /digital_twin/boxes)
  detections_topic : bridged Detection3DArray   (default: /detected_cups)
  target_frame     : marker / detection frame   (default: world)
  threshold        : occupancy overlap threshold (default: 0.2)
  use_test_pub     : true → run synthetic test_pub instead of the bridge
  tuner            : true|false — show the p_start/v_dir tuner UI (default: true)
  sync_pyramid_geometry : true|false — verifier polls FastAPI for cp/degree (default: true)
  pyramid_config_url    : GET endpoint for the pyramid config
  cp_z             : perceived L1 cup-top height in world frame (default: 0.14)
  sync_poll_period_s    : seconds between geometry polls (default: 5.0)
  cp_offset_x/y    : static exo→base XY nudge added on top of cp so slot boxes
                     land on the (offset) detected cups (default: 0.0)
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
        # /stack namespaced so the integration aggregator_node sits in front of
        # GSP. /stack_track_ids stays as-is (plan_executor consumes it directly).
        remappings=[('/stack', '/vision/stack')],
        parameters=[{
            'target_frame': target_frame,
            'threshold': threshold,
            # Runtime geometry sync (verifier mirrors FastAPI pyramid center/degree).
            'sync_pyramid_geometry': LaunchConfiguration('sync_pyramid_geometry'),
            'pyramid_config_url': LaunchConfiguration('pyramid_config_url'),
            'cp_z': LaunchConfiguration('cp_z'),
            'sync_poll_period_s': LaunchConfiguration('sync_poll_period_s'),
            # Static exo→base XY nudge added on top of the synced cp (the exo
            # world frame is offset from base_link; placed cups read ~+x/-y).
            'cp_offset_x': LaunchConfiguration('cp_offset_x'),
            'cp_offset_y': LaunchConfiguration('cp_offset_y'),
            'cp_offset_z': LaunchConfiguration('cp_offset_z'),
        }])

    logger = Node(
        package='cup_stacking_verify', executable='topic_logger',
        name='topic_logger_node', output='screen',
        parameters=[{'detections_topic': detections_topic}])

    tuner = Node(
        package='cup_stacking_verify', executable='pose_tuner',
        name='pose_tuner_node', output='screen',
        condition=IfCondition(LaunchConfiguration('tuner')),
        parameters=[{'target_node': 'cup_occupancy_verifier'}])

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
        DeclareLaunchArgument('threshold', default_value='0.2'),
        DeclareLaunchArgument('use_test_pub', default_value='false'),
        DeclareLaunchArgument('tuner', default_value='true'),
        DeclareLaunchArgument('sync_pyramid_geometry', default_value='true'),
        DeclareLaunchArgument(
            'pyramid_config_url',
            default_value='http://localhost/api/robot/config/pyramid'),
        DeclareLaunchArgument('cp_z', default_value='0.14'),
        DeclareLaunchArgument('sync_poll_period_s', default_value='5.0'),
        DeclareLaunchArgument('cp_offset_x', default_value='0.0'),
        DeclareLaunchArgument('cp_offset_y', default_value='0.0'),
        DeclareLaunchArgument('cp_offset_z', default_value='0.0'),
        bridge,
        test_pub,
        verifier,
        logger,
        tuner,
        rviz_node,
    ])
