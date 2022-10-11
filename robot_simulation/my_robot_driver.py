from math import sqrt
import rclpy
from geometry_msgs.msg import Twist

HALF_WIDTH = (497/1000)/2
HALF_LENGTH = HALF_WIDTH
ROBOT_ROTATION_RADIUS = sqrt(HALF_LENGTH ** 2 + HALF_WIDTH ** 2)
WHEEL_RADIUS = 0.11
MAX_VELOCITY = 6.4
MAX_TORQUE = 20
WEIGHT = 8.8

class MyRobotDriver:
        def init(self, webots_node, properties):
            self.__robot = webots_node.robot
            
            self.__front_left_motor = self.__robot.getDevice('left wheel')
            #self.__back_left_motor = self.__robot.getDevice('back left wheel')
            self.__front_right_motor = self.__robot.getDevice('right wheel')
            #self.__back_right_motor = self.__robot.getDevice('back right wheel')

            self.__front_left_motor.setPosition(float('inf'))
            self.__front_left_motor.setVelocity(0)

            #self.__back_left_motor.setPosition(float('inf'))
            #self.__back_left_motor.setVelocity(0)

            self.__front_right_motor.setPosition(float('inf'))
            self.__front_right_motor.setVelocity(0)

            #self.__back_right_motor.setPosition(float('inf'))
            #self.__back_right_motor.setVelocity(0)

            self.__target_twist = Twist()
            
            rclpy.init(args=None)
            self.__node = rclpy.create_node('my_robot_driver')
            self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

        def __cmd_vel_callback(self, twist):
            self.__target_twist = twist

        def step(self):
            rclpy.spin_once(self.__node, timeout_sec=0)

            forward_speed = self.__target_twist.linear.x
            angular_speed = self.__target_twist.angular.z

            command_motor_left = (forward_speed - angular_speed * ROBOT_ROTATION_RADIUS) / WHEEL_RADIUS
            command_motor_left = min(command_motor_left, MAX_VELOCITY)
            command_motor_right = (forward_speed + angular_speed * ROBOT_ROTATION_RADIUS) / WHEEL_RADIUS
            command_motor_right = min(command_motor_right, MAX_VELOCITY)

            self.__front_left_motor.setVelocity(command_motor_left)
            #self.__back_left_motor.setVelocity(command_motor_left)
            self.__front_right_motor.setVelocity(command_motor_right)
            #self.__back_right_motor.setVelocity(command_motor_right)
