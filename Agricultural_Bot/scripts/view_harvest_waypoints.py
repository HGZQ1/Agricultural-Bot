#!/usr/bin/env python3
"""Display the installed harvest mission in RViz without commanding motion.

Run from the project root after sourcing both workspaces. Add an RViz
MarkerArray display for /harvest_waypoint_markers with Reliable and
Transient Local QoS. Arrows show parked vehicle yaw, including reverse stops.
"""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from visualization_msgs.msg import Marker, MarkerArray

import yaml


def build_markers(route):
    """Build labels and yaw arrows for work stops, transits and home."""
    items = []
    for waypoint in sorted(route['waypoints'], key=lambda p: p['sequence']):
        if not waypoint.get('enabled', True):
            continue
        left = waypoint['work_side'] == 'left'
        label = f"{waypoint['sequence']:02d}:{'L' if left else 'R'}"
        color = (0.1, 0.5, 1.0) if left else (1.0, 0.45, 0.1)
        items.append((label, waypoint['base_pose'], color))
    for index, transit in enumerate(route.get('transits', []), 1):
        items.append((f'H{index}', transit['pose'], (0.7, 0.2, 1.0)))
    items.append(('HOME', route['home']['pose'], (0.1, 0.9, 0.2)))
    markers = MarkerArray()
    for label, pose, color in items:
        for marker_type in (Marker.ARROW, Marker.TEXT_VIEW_FACING):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns = 'harvest_waypoints'
            marker.id = len(markers.markers)
            marker.type = marker_type
            marker.action = Marker.ADD
            marker.pose.position.x = float(pose['x'])
            marker.pose.position.y = float(pose['y'])
            marker.pose.position.z = 0.08
            marker.pose.orientation.z = math.sin(float(pose['yaw']) / 2)
            marker.pose.orientation.w = math.cos(float(pose['yaw']) / 2)
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            if marker_type == Marker.ARROW:
                marker.scale.x, marker.scale.y, marker.scale.z = (
                    0.30, 0.04, 0.06)
            else:
                marker.text = label
                marker.scale.z = 0.14
                marker.pose.position.y += (
                    -0.16 if ':R' in label else 0.16)
                marker.pose.position.z = 0.30
            markers.markers.append(marker)
    return markers


def main():
    """Publish display markers without navigation or lock commands."""
    share = Path(get_package_share_directory('agri_task_manager'))
    mission = (
        share / 'config/missions/tomato_field_default_single_corridor.yaml')
    route = yaml.safe_load(mission.read_text(encoding='utf-8'))['route']
    markers = build_markers(route)
    harvest_count = sum(w.get('enabled', True) for w in route['waypoints'])
    transit_count = len(route.get('transits', []))
    rclpy.init()
    node = Node('harvest_waypoint_view', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    qos = QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE)
    publisher = node.create_publisher(
        MarkerArray, '/harvest_waypoint_markers', qos)
    node.create_timer(1.0, lambda: publisher.publish(markers))
    publisher.publish(markers)
    print(
        f'{harvest_count} harvest stops + {transit_count} transits + HOME '
        f'-> /harvest_waypoint_markers\nMission: {mission}', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
