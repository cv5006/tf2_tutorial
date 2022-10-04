import math
import queue

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Tracer(Node):
    def __init__(self):
        super().__init__('tracer')
        self.tf_buff = Buffer()
        self.tf_listener = TransformListener(self.tf_buff, self)

        self.timer = self.create_timer(0.01, self.TimerCallback)
        self.trace_pub = self.create_publisher(PoseStamped, '/arrow', 10)
        self.path_msg = Path()
        self.path_pub = self.create_publisher(Path, '/path', 10)

    def TimerCallback(self):
        try:
            t = self.tf_listener
            t = self.tf_buff.lookup_transform(
                'world',
                'tip',
                rclpy.time.Time()
            )
            
            trace = PoseStamped()
            trace.header.stamp = self.get_clock().now().to_msg()
            trace.header.frame_id = 'world'
            trace.pose.position.x = t.transform.translation.x
            trace.pose.position.y = t.transform.translation.y
            trace.pose.position.z = t.transform.translation.z
            trace.pose.orientation.x = t.transform.rotation.x
            trace.pose.orientation.y = t.transform.rotation.y
            trace.pose.orientation.z = t.transform.rotation.z
            trace.pose.orientation.w = t.transform.rotation.w
            self.trace_pub.publish(trace)

            self.path_msg.poses.append(trace)
            if len(self.path_msg.poses) > 20:
                self.path_msg.poses.pop(0)
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            self.path_msg.header.frame_id = 'world'
            self.path_pub.publish(self.path_msg)
            print('pub')


        except TransformException as ex:
            self.get_logger().error(f'tf fucked up: {ex}')


def main():
    rclpy.init()
    
    node = Tracer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn('stop')
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
