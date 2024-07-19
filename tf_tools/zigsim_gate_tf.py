import rclpy
from rclpy.node import Node
import logging
# msgs
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
# add
import numpy as np
from builtin_interfaces.msg import Time
import socket
import json
from tf2_ros import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster

class ZigsimGateTF(Node):
    def __init__(self):
        super().__init__('zigsim_gate_tf')
        self.init_param()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(self.publish_interval, self.CB_timer)

    def CB_timer(self):
        # スマホアプリによってここ変える
        data, addr = self.sock.recvfrom(1024)  # 1024バイトまでのデータを受け取る
        json_data = json.loads(data.decode('utf-8'))
        qx,qy,qz,qw = json_data["sensordata"]["quaternion"]["x"],\
                        json_data["sensordata"]["quaternion"]["y"],\
                        json_data["sensordata"]["quaternion"]["z"],\
                        json_data["sensordata"]["quaternion"]["w"]
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id_parent
        t.child_frame_id = self.frame_id_child

        t.transform.translation.x = self.distance * (1 - 2*qz*qz)
        t.transform.translation.y = -1 * self.distance * 2*qz*qw
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qw
        t.transform.rotation.w = qz
        self.tf_broadcaster.sendTransform(t)
        

    def init_param(self):
        self.declare_parameter('UDP_IP', "0.0.0.0")
        self.declare_parameter('UDP_PORT', 50000)
        self.declare_parameter('frame_id_parent',"map")
        self.declare_parameter('frame_id_child',"camera_link")
        self.declare_parameter('publish_interval', 0.01)
        self.declare_parameter('distance', 0.7)

        self.UDP_IP = self.get_parameter('UDP_IP').get_parameter_value().string_value
        self.UDP_PORT = self.get_parameter('UDP_PORT').get_parameter_value().integer_value
        self.frame_id_parent = self.get_parameter('frame_id_parent').get_parameter_value().string_value
        self.frame_id_child = self.get_parameter('frame_id_child').get_parameter_value().string_value
        self.publish_interval = self.get_parameter("publish_interval").get_parameter_value().double_value
        self.distance = self.get_parameter("distance").get_parameter_value().double_value

def main():
    rclpy.init()
    node = ZigsimGateTF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()