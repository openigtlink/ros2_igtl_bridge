from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    igtl_bridge = Node(
        package="ros2_igtl_bridge",
        executable="igtl_node",
        parameters=[
            {"RIB_server_ip": LaunchConfiguration('ip')},
            {"RIB_port": LaunchConfiguration('port')},
            {"RIB_type": LaunchConfiguration('mode')}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value = "server",
            description = "OpenIGTLBridge Mode: client or server"
        ),
        actions.LogInfo(msg = ["mode: ", LaunchConfiguration('mode')]),

        DeclareLaunchArgument(
            "port",
            default_value = "18944",
            description = "OpenIGTLBridge port number"
        ),
        actions.LogInfo(msg = ["port: ", LaunchConfiguration('port')]),

        DeclareLaunchArgument(
            "ip",
            default_value = "192.168.2.21",
            description = "OpenIGTLBridge IP address"
        ),
        actions.LogInfo(msg = ["ip: ", LaunchConfiguration('ip')]),        
        igtl_bridge
    ])