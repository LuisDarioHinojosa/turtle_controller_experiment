from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    turtlesimNode = Node(package = "turtlesim",executable = "turtlesim_node")
    controllerNode = Node(package = "controller_main",executable = "controller_node")
    ld.add_action(turtlesimNode)
    ld.add_action(controllerNode)
    return ld