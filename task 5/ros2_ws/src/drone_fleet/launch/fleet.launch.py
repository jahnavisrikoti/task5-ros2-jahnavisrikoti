"""
fleet.launch.py
Starts the full drone fleet simulation:
  • Alpha  drone (battery: 100%)
  • Beta   drone (battery:  60%)
  • Gamma  drone (battery:  35% — starts near critical)
  • Fleet manager (monitors all three)
  • Health monitor (tracks drain rates)

Usage:
    ros2 launch drone_fleet fleet.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── Alpha ──────────────────────────────────────────────────────────────
        Node(
            package    = 'drone_fleet',
            executable = 'drone_node',
            name       = 'drone_alpha',
            output     = 'screen',
            parameters = [{
                'drone_name':      'Alpha',
                'initial_battery': 100.0,
                'mission_name':    'Alpha-Recon',
            }],
        ),

        # ── Beta ───────────────────────────────────────────────────────────────
        Node(
            package    = 'drone_fleet',
            executable = 'drone_node',
            name       = 'drone_beta',
            output     = 'screen',
            parameters = [{
                'drone_name':      'Beta',
                'initial_battery': 60.0,
                'mission_name':    'Beta-Survey',
            }],
        ),

        # ── Gamma (starts nearly critical at 35%) ─────────────────────────────
        Node(
            package    = 'drone_fleet',
            executable = 'drone_node',
            name       = 'drone_gamma',
            output     = 'screen',
            parameters = [{
                'drone_name':      'Gamma',
                'initial_battery': 35.0,
                'mission_name':    'Gamma-Patrol',
            }],
        ),

        # ── Fleet Manager ─────────────────────────────────────────────────────
        Node(
            package    = 'drone_fleet',
            executable = 'fleet_manager',
            name       = 'fleet_manager',
            output     = 'screen',
        ),

        # ── Health Monitor ────────────────────────────────────────────────────
        Node(
            package    = 'drone_fleet',
            executable = 'health_monitor',
            name       = 'health_monitor',
            output     = 'screen',
        ),

    ])
