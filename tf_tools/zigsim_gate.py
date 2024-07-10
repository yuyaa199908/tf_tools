import rclpy
from rclpy.node import Node
import logging
# msgs
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
# add
import numpy as np
from builtin_interfaces.msg import Time
import socket
import json


class ZigsimGate(Node):
    def __init__(self):
        super().__init__('zigsim_gate')
        self.init_param()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))

        self.pub_pose = self.create_publisher(PoseStamped, 'iphone_pose', 10)
        self.timer = self.create_timer(self.publish_interval, self.CB_timer)

    def CB_timer(self):
        # スマホアプリによってここ変える
        data, addr = self.sock.recvfrom(1024)  # 1024バイトまでのデータを受け取る
        json_data = json.loads(data.decode('utf-8'))

        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        now = self.get_clock().now()
        msg.header.stamp = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = json_data["sensordata"]["quaternion"]["x"]
        msg.pose.orientation.y = json_data["sensordata"]["quaternion"]["y"]
        msg.pose.orientation.z = json_data["sensordata"]["quaternion"]["z"]
        msg.pose.orientation.w = json_data["sensordata"]["quaternion"]["w"]
        self.pub_pose.publish(msg)


        # data, addr = self.sock.recvfrom(8192)  # 1024バイトまでのデータを受け取る
        # tmp = data.decode('utf-8').split(",")
        # if tmp[0].split('\t')[1] != "GROTV":
        #     msg = PoseStamped()
        #     msg.header.frame_id = "map"
        #     now = self.get_clock().now()
        #     msg.header.stamp = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])

        #     msg.pose.position.x = 0.0
        #     msg.pose.position.y = 0.0
        #     msg.pose.position.z = 0.0

        #     msg.pose.orientation.x = float(tmp[1])
        #     msg.pose.orientation.y = float(tmp[2])
        #     msg.pose.orientation.z = float(tmp[3])
        #     msg.pose.orientation.w = - float(tmp[4])
        #     self.pub_pose.publish(msg)
        # self.get_logger().info('Publishing: ""' % msg)

    def init_param(self):
        self.declare_parameter('UDP_IP', "0.0.0.0")
        self.declare_parameter('UDP_PORT', 50000)
        self.declare_parameter('frame_id', "hoge")
        self.declare_parameter('publish_interval', 0.5)

        self.UDP_IP = self.get_parameter('UDP_IP').get_parameter_value().string_value
        self.UDP_PORT = self.get_parameter('UDP_PORT').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_interval = self.get_parameter("publish_interval").get_parameter_value().double_value


def main():
    rclpy.init()
    node = ZigsimGate()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()