

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped

import math
import transforms3d
from datetime import datetime
from pyubx2 import UBXMessage

from .gps_pub import GpsPub


class NavPub(GpsPub):

    FIX_TYPE_NO_FIX = 0
    FIX_TYPE_DEAD_RECKONING_ONLY = 1
    FIX_TYPE_2D = 2
    FIX_TYPE_3D = 3
    FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED = 4
    FIX_TYPE_TIME_ONLY = 5

    def __init__(self, node: Node):
        super().__init__(node)

    def _work(self, parsed_data: UBXMessage) -> None:

        if parsed_data.identity == "NAV-PVT":

            fix_position = NavSatFix()

            # header
            fix_position.header.frame_id = self.frame_id

            # stamp
            if bool(parsed_data.validTime) and bool(parsed_data.validDate) and bool(parsed_data.fullyResolved):

                dt = datetime.now()
                dt.replace(day=parsed_data.day, month=parsed_data.month, year=parsed_data.month,
                           hour=parsed_data.hour, minute=parsed_data.min,  second=parsed_data.second)

                nano = int(parsed_data.nano)

                if nano < 0:
                    fix_position.header.stamp.sec = int(dt.timestamp()) - 1
                    fix_position.header.stamp.nanosec = nano + int(1e9)
                else:
                    fix_position.header.stamp.sec = int(dt.timestamp())
                    fix_position.header.stamp.nanosec = nano

            else:
                fix_position.header.stamp = self.node.get_clock().now().to_msg()

            # postion
            fix_position.latitude = parsed_data.lat
            fix_position.longitude = parsed_data.lon
            fix_position.altitude = parsed_data.height * float(1e-3)

            # status
            valid_fix = bool(parsed_data.gnssFixOk)
            fix_type = int(parsed_data.fixType)

            if valid_fix and fix_type >= self.FIX_TYPE_2D:
                fix_position.status.status = NavSatStatus.STATUS_FIX

                if bool(parsed_data.carrSoln):
                    fix_position.status.status = NavSatStatus.STATUS_GBAS_FIX

            else:
                fix_position.status.status = NavSatStatus.STATUS_NO_FIX

            fix_position.status.service = NavSatStatus.SERVICE_GPS

            # covariances
            var_h = pow(parsed_data.hAcc / 1000.0, 2)
            var_v = pow(parsed_data.vAcc / 1000.0, 2)
            fix_position.position_covariance[0] = var_h
            fix_position.position_covariance[4] = var_h
            fix_position.position_covariance[8] = var_v
            fix_position.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            self.fix_pub.publish(fix_position)

            # twist
            current_vel = TwistWithCovarianceStamped()
            current_vel.header.frame_id = self.frame_id
            current_vel.header.stamp = fix_position.header.stamp

            # vels
            current_vel.twist.twist.linear.x = parsed_data.velE * float(1e-3)
            current_vel.twist.twist.linear.y = parsed_data.velN * float(1e-3)
            current_vel.twist.twist.linear.z = -parsed_data.velD * float(1e-3)

            # covariances
            cov_speed = pow(parsed_data.sAcc * 1e-3, 2)
            cols = 6
            current_vel.twist.covariance[cols * 0 + 0] = cov_speed
            current_vel.twist.covariance[cols * 1 + 1] = cov_speed
            current_vel.twist.covariance[cols * 2 + 2] = cov_speed
            current_vel.twist.covariance[cols * 3 + 3] = -1

            self.vel_pub.publish(current_vel)

        elif parsed_data.identity == "NAV-RELPOSNED":

            heading_msg = Imu()
            heading_msg.header.frame_id = self.frame_id
            heading_msg.header.stamp = self.node.get_clock().now().to_msg()

            heading_msg.linear_acceleration_covariance[0] = -1
            heading_msg.angular_velocity_covariance[0] = -1

            heading = (float(parsed_data.relPosHeading) *
                       float(1e-5) / 180.0 * math.pi) - math.pi * 2
            orientation = transforms3d.euler.euler2quat(0, 0, heading)
            heading_msg.orientation.x = orientation[0]
            heading_msg.orientation.y = orientation[1]
            heading_msg.orientation.z = orientation[2]
            heading_msg.orientation.w = orientation[3]
            heading_msg.orientation_covariance[0] = 1000.0
            heading_msg.orientation_covariance[4] = 1000.0
            heading_msg.orientation_covariance[8] = 1000.0

            self.heading_pub.publish(heading_msg)
