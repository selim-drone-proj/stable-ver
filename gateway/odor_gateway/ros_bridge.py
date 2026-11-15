"""
ros_bridge.py - 게이트웨이 JSON 기록을 ROS 2 메시지로 발행 (rclpy, odor_msgs 필요)

    /station/<id>/sample   odor_msgs/StationSample
    /health/<id>           odor_msgs/DeviceHealth   (관측소, 게이트웨이 gw_01)
    /odor/sample           odor_msgs/OdorSample     (이동형 LoRa 요약. 기체 안의 토픽과 이름이 같다.
                                                    기체와 지상국을 같은 ROS 네트워크에 두면 둘 다 보이므로 ROS_DOMAIN_ID 를 나눌 것)

odor_msgs 는 moving-ver/ros2_ws/src/odor_msgs 를 같은 워크스페이스에 넣고 빌드한다.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from odor_msgs.msg import DeviceHealth, OdorSample, StationSample

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
        elif rec["type"] == "odor_sample":
            cls, topic = OdorSample, "/odor/sample"
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
