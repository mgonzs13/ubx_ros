
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    update_rate = LaunchConfiguration("update_rate")
    update_rate_cmd = DeclareLaunchArgument(
        "update_rate",
        default_value="30",
        description="update rate")

    device = LaunchConfiguration("device")
    device_cmd = DeclareLaunchArgument(
        "device",
        default_value="/dev/ttyACM0",
        description="GPS device")

    baud = LaunchConfiguration("baud")
    baud_cmd = DeclareLaunchArgument(
        "baud",
        default_value="460800",
        description="baud rate")

    frame_id = LaunchConfiguration("frame_id")
    frame_id_cmd = DeclareLaunchArgument(
        "frame_id",
        default_value="gps",
        description="frame_id")

    use_nav = LaunchConfiguration("use_nav")
    use_nav_cmd = DeclareLaunchArgument(
        "use_nav",
        default_value="True",
        description="Wheter to use NAV messages from UBX protocol")

    ubx_node = Node(package="ubx_ros",
                    executable="ubx_node",
                    output="both",
                    parameters=[
                        {
                            "update_rate": update_rate,
                            "device": device,
                            "baud": baud,
                            "frame_id": frame_id,
                            "use_nav": use_nav
                        }
                    ]
                    )

    ntrip_client_node = Node(
        name="ntrip_client_node",
        package="ntrip_client",
        executable="ntrip_ros.py",
        parameters=[
            {
                "host": os.getenv("NTRIP_HOST"),
                "port": int(os.getenv("NTRIP_PORT")),
                "mountpoint": os.getenv("NTRIP_MOUNTPOINT"),

                "authenticate": True,
                "username": os.getenv("NTRIP_USER"),
                "password": os.getenv("NTRIP_PASSWORD"),

                "rtcm_frame_id": "gps"
            }
        ],
        remappings=[
            ("nmea", "gps/nmea"),
            ("rtcm", "gps/rtcm")
        ],
    )

    ld = LaunchDescription()

    ld.add_action(update_rate_cmd)
    ld.add_action(device_cmd)
    ld.add_action(baud_cmd)
    ld.add_action(frame_id_cmd)
    ld.add_action(use_nav_cmd)

    ld.add_action(ubx_node)
    ld.add_action(ntrip_client_node)

    return ld
