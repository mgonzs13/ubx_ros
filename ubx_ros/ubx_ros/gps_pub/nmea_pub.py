
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped

import math
from pyubx2 import UBXMessage


from .gps_pub import GpsPub
from .gps_qualities import GpsQualities


class NmeaPub(GpsPub):

    def __init__(self, node: Node):
        super().__init__(node)

        self.gps_qualities = GpsQualities(node)
        self.valid_fix = False
        self.std_data = []

    def _work(self, parsed_data: UBXMessage) -> None:

        stamp = self.node.get_clock().now().to_msg()

        # fix position
        if parsed_data.identity == "GNGGA":

            fix_position = NavSatFix()

            # header
            fix_position.header.frame_id = self.frame_id
            fix_position.header.stamp = stamp

            # quality
            fix_type = parsed_data.quality
            if not (fix_type in self.gps_qualities):
                fix_type = -1

            gps_qual = self.gps_qualities[fix_type]

            # status
            fix_position.status.service = NavSatStatus.SERVICE_GPS
            fix_position.status.status = gps_qual[1]

            if fix_position.status.status >= 0:
                self.valid_fix = True
            else:
                self.valid_fix = False

            # covariance
            fix_position.position_covariance_type = gps_qual[2]

            if self.std_data:
                hdop = parsed_data.HDOP
                fix_position.position_covariance[0] = (
                    hdop * self.std_data[1]) ** 2
                fix_position.position_covariance[4] = (
                    hdop * self.std_data[0]) ** 2
                fix_position.position_covariance[8] = (
                    2 * hdop * self.std_data[2]) ** 2

            # positions
            fix_position.latitude = parsed_data.lat
            fix_position.longitude = parsed_data.lon
            fix_position.altitude = parsed_data.alt
            self.fix_pub.publish(fix_position)

        # std dev data
        elif parsed_data.identity == "GNGST":
            self.std_data = [parsed_data.stdLat,
                             parsed_data.stdLong,
                             parsed_data.stdAlt]

        # velocity
        elif parsed_data.identity == "GNVTG":
            if self.valid_fix:

                KH_2_MS = 1000/(60 * 60)

                current_vel = TwistWithCovarianceStamped()
                current_vel.header.frame_id = self.frame_id
                current_vel.header.stamp = stamp

                current_vel.twist.twist.linear.x = float(parsed_data.sogk) * \
                    math.sin(float(parsed_data.cogt)) * KH_2_MS
                current_vel.twist.twist.linear.y = float(parsed_data.sogk) * \
                    math.cos(float(parsed_data.cogt)) * KH_2_MS
                self.vel_pub.publish(current_vel)
