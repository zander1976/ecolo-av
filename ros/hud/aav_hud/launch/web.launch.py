"""
The Orin's web page (aav_hud's `aav_web`), read only: no window, no publisher.

  ros2 launch aav_hud web.launch.py                          the car: 127.0.0.1:8780
  ros2 launch aav_hud web.launch.py bind:=0.0.0.0            laptops on the car's network
  ros2 launch aav_hud web.launch.py profile:=virtual esp32:=ws://127.0.0.1:8766/ws

Three tabs: CAR (each sensor and actuator), HOPS (what ROS asked, what CORTEX wrote, what
the ESP32 got, 60 s graphs) and ROS (every topic against docs/interfaces.yaml for the
profile, as pixi run check). Topic names: hw_panel's built-in defaults (its subscriptions).
esp32: the ESP32's WebSocket of status lines (the board's, or the virtual car's sim_esp32).
Design: docs/designs/2026-11-27_web-screens.md.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def web_node(context):
    """Make the node; the address, the profile and the ESP32 are its arguments."""
    arg = lambda name: LaunchConfiguration(name).perform(context).strip()  # noqa: E731
    arguments = ['--bind', arg('bind'), '--port', arg('port'), '--profile', arg('profile')]
    if arg('esp32'):
        arguments += ['--esp32', arg('esp32')]
    return [Node(
        package='aav_hud',
        executable='aav_web',
        name='web',
        arguments=arguments,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bind', default_value='127.0.0.1',
                              description='The address to serve on: 0.0.0.0 for every '
                                          'computer on the network'),
        DeclareLaunchArgument('port', default_value='8780', description='The HTTP port'),
        DeclareLaunchArgument('profile', default_value='car',
                              description='The ROS tab judges against this profile of '
                                          'docs/interfaces.yaml (car, virtual, sim, ...)'),
        DeclareLaunchArgument('esp32', default_value='',
                              description="The ESP32's WebSocket of status lines, such as "
                                          'ws://192.168.4.1/ws. Empty: none'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=web_node),
    ])
