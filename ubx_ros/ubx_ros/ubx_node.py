#!/usr/bin/env python3


from serial import Serial
from pyubx2 import UBXReader
from pyubx2 import UBXMessage, SET

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from mavros_msgs.msg import RTCM

from ubx_ros.gps_pub import NmeaPub, NavPub


class UbxNode(Node):

    def __init__(self) -> None:
        super().__init__("ubx_node")

        # params
        device = self.declare_parameter("device", "/dev/ttyACM0").value
        baud_rate = self.declare_parameter("baud", 9600).value
        self.use_nav = self.declare_parameter("use_nav", True).value

        # serial
        self.serial = Serial(device, baud_rate, timeout=5)
        self.ubr = UBXReader(self.serial)
        self.config()

        # pubs
        self.gps_pub = None
        if self.use_nav:
            self.gps_pub = NavPub(self)
        else:
            self.gps_pub = NmeaPub(self)

        # subs
        self.rtcm_sub = self.create_subscription(
            RTCM, "rtcm", self.rtcm_cb, 10)

        # wall timer
        self.lista = []
        self.create_timer(0.001, self.work)

    def config(self) -> None:
        cfg_msg = UBXMessage(0x06, 0x17, SET,
                             posFilt=1, mskPosFilt=0, timeFilt=0, dateFilt=0, gpsOnlyFilter=0, trackFilt=0,
                             compat=0, consider=0, limit82=0, highPrec=1,
                             gps=0, sbas=0, galileo=0, qzss=0, glonass=0, beidou=0,
                             svNumbering=0, nmeaVersion=3, numSV=0,
                             mainTalkerId=0, gsvTalkerId=0, bdsTalkerId=b'',
                             version=0, reserved1=0)
        self.serial.write(cfg_msg.serialize())

        # enabling msgs
        self.enable_msgs(0xf0, 0x07, enable=(not self.use_nav))  # GNGST
        self.enable_msgs(0xf0, 0x0e, enable=(not self.use_nav))  # GNTHS
        self.enable_msgs(0xf0, 0xa0, enable=(not self.use_nav))  # GPDTM

        self.enable_msgs(0x0a, 0x09, enable=self.use_nav)  # MON-HW
        self.enable_msgs(0x01, 0x35, enable=self.use_nav)  # NAV-SAT
        self.enable_msgs(0x01, 0x22, enable=self.use_nav)  # NAV-CLOCK
        self.enable_msgs(0x01, 0x03, enable=self.use_nav)  # NAV-STATUS
        self.enable_msgs(0x01, 0x01, enable=self.use_nav)  # NAV-POSECEF
        self.enable_msgs(0x01, 0x3c, enable=self.use_nav)  # NAV-RELPOSNED
        self.enable_msgs(0x01, 0x07, enable=self.use_nav)  # NAV-PVT

    def enable_msgs(self, msg_class: int, msg_id: int, enable: bool = True) -> None:
        cfg_msg = UBXMessage(
            0x06, 0x01, SET,
            msgClass=msg_class, msgID=msg_id,
            rateDDC=0, rateUART1=0, rateUART2=0, rateSPI=0,
            rateUSB=int(enable))
        self.serial.write(cfg_msg.serialize())

    def work(self) -> None:
        (raw_data, parsed_data) = self.ubr.read()
        self.gps_pub.work(raw_data, parsed_data)

    def rtcm_cb(self, msg: RTCM) -> None:
        self.serial.write(msg.data)

    def destroy_node(self) -> None:
        super.destroy_node()
        self.serial.close()


def main(args=None):
    rclpy.init(args=args)

    node = UbxNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
