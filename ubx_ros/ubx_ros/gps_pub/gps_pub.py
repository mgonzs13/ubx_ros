
from abc import ABC, abstractmethod
from pyubx2 import UBXMessage

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from nmea_msgs.msg import Sentence


class GpsPub(ABC):

    def __init__(self, node: Node) -> None:

        self.node = node

        self.frame_id = self.node.declare_parameter("frame_id", "gps").value

        self.nmea_pub = self.node.create_publisher(Sentence, "gps/nmea", 10)
        self.fix_pub = self.node.create_publisher(NavSatFix, "gps/fix", 10)
        self.vel_pub = self.node.create_publisher(
            TwistWithCovarianceStamped, "gps/fix_velocity", 10)
        self.heading_pub = self.node.create_publisher(Imu, "gps/heading", 10)

    def work(self, raw_data: str, parsed_data: UBXMessage) -> None:

        raw_data = raw_data.decode("utf-8", errors="replace")
        if raw_data[0] == "$" or raw_data[0] == "!":
            stamp = self.node.get_clock().now().to_msg()

            sentence_msg = Sentence()
            sentence_msg.header.frame_id = self.frame_id
            sentence_msg.header.stamp = stamp
            sentence_msg.sentence = raw_data
            self.nmea_pub.publish(sentence_msg)

        self._work(parsed_data)

    @abstractmethod
    def _work(self, parsed_data: UBXMessage) -> None:
        pass
