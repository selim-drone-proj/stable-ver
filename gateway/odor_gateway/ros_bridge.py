"""
ros_bridge.py - 게이트웨이 JSON 기록을 ROS 2 메시지로 발행 (rclpy, odor_msgs 필요)

    /station/<id>/sample   odor_msgs/StationSample
    /health/<id>           odor_msgs/DeviceHealth   (관측소, 게이트웨이 gw_01)

odor_msgs 는 moving-ver/ros2_ws/src/odor_msgs 를 같은 워크스페이스에 넣고 빌드한다.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from odor_msgs.msg import DeviceHealth, StationSample

from .odor_json import fill_msg


class RosBridge:
    def __init__(self, node_name: str = "lora_gateway"):
        rclpy.init()
        self.node = Node(node_name)
        self.pubs = {}
        self.node.get_logger().info("lora_gateway ROS 발행 시작")

    def _pub(self, cls, topic: str):
        pub = self.pubs.get(topic)
        if pub is None:
            pub = self.node.create_publisher(cls, topic, 10)
            self.pubs[topic] = pub
        return pub

    def publish(self, rec: dict):
        if rec["type"] == "station_sample":
            cls, topic = StationSample, f"/station/{rec['device_id']}/sample"
        elif rec["type"] == "device_health":
            cls, topic = DeviceHealth, f"/health/{rec['device_id']}"
        else:
            return
        self._pub(cls, topic).publish(fill_msg(cls(), rec))

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self):
        self.node.destroy_node()
        rclpy.try_shutdown()
