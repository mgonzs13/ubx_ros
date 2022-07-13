

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from typing import List, Union


class GpsQualities:

    def __init__(self, node: Node) -> None:

        # params
        self.default_epe_quality0 = node.declare_parameter(
            "epe_quality0", 1000000).value
        self.default_epe_quality1 = node.declare_parameter(
            "epe_quality1", 4.0).value
        self.default_epe_quality2 = node.declare_parameter(
            "epe_quality2", 0.1).value
        self.default_epe_quality4 = node.declare_parameter(
            "epe_quality4", 0.02).value
        self.default_epe_quality5 = node.declare_parameter(
            "epe_quality5", 4.0).value
        self.default_epe_quality9 = node.declare_parameter(
            "epe_quality9", 3.0).value

        # qualities
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

    def __getitem__(self, n: int) -> List[Union[int, float]]:
        return self.gps_qualities[n]

    def __contains__(self, value: int) -> bool:
        return value in self.gps_qualities
