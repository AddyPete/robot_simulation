from math import sin, cos, pi
from re import M
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformBroadcaster, TransformStamped, StaticTransformBroadcaster
from tf_transformations import euler_from_quaternion

ODOM_FRAME_NAME = 'odom'
BASE_LINK_FRAME_NAME = 'base_link'
ODOM_TOPIC_NAME = '/odom0'

RATE_HZ = 20

class OdomEstimator(Node):

    def __init__(self):
        super().__init__('odom_estimator')

        qos_profile = QoSProfile(depth=10)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.odom_pub = self.create_publisher(Odometry, ODOM_TOPIC_NAME, qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        self.last_time = self.get_clock().now()
        self.last_twist = Twist()
        self.last_twist.linear.x = 0.0
        self.last_twist.linear.y = 0.0
        self.last_twist.linear.z = 0.0
        self.last_twist.angular.x = 0.0
        self.last_twist.angular.y = 0.0
        self.last_twist.angular.z = 0.0
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0
        self.vel_subscription = self.create_subscription(Twist, '/cmd_vel',
            self.vel_listener_callback, qos_profile)
        self.timer = self.create_timer(1 / float(RATE_HZ), self.timer_callback)
    
    def timer_callback(self):
        now_time = self.get_clock().now()
        self.make_odom_transform_and_msg(now_time)

    def vel_listener_callback(self, twist_msg):
        self.last_twist = twist_msg
        pass

    def make_odom_transform_and_msg(self, now_time):
        delta_time = float((now_time - self.last_time).nanoseconds) / (10 ** 9)
        self.last_time = now_time
        self.theta += delta_time * self.last_twist.angular.z
        delta_distance = delta_time * self.last_twist.linear.x
        self.x_pos += delta_distance * cos(self.theta)
        self.y_pos += delta_distance * sin(self.theta)
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        #quat = tf_transformations.quaternion_from_euler(0, 0, 2)

        odom_msg = Odometry()
        odom_msg.header.frame_id = ODOM_FRAME_NAME
        odom_msg.child_frame_id = BASE_LINK_FRAME_NAME
        odom_msg.header.stamp = now_time.to_msg()
        odom_msg.pose.pose.position.x = self.x_pos
        odom_msg.pose.pose.position.y = self.y_pos
        odom_msg.pose.pose.position.z = 0.0
        #odom_msg.pose.pose.position.z = 5.0
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.twist.twist.linear.x = self.last_twist.linear.x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.last_twist.angular.z
        
        self.odom_pub.publish(odom_msg)

def main():
    rclpy.init(args=None)
    odom_estimator = OdomEstimator()
    rclpy.spin(odom_estimator)
    odom_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
