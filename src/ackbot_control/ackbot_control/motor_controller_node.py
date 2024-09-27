#!/usr/bin/env python3

# public lib
import sys
import math
import threading
from math import pi
from time import sleep
from Rosmaster_Lib import Rosmaster

# ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, JointState
from rclpy.clock import Clock

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Car initialization
        self.car = Rosmaster()
        self.car.set_car_type(5)

        # ROS Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int32, 'RGBLight', self.rgb_light_callback, 100)
        self.create_subscription(Bool, 'Buzzer', self.buzzer_callback, 100)

        # ROS Publishers
        self.voltage_publisher = self.create_publisher(Float32, 'voltage', 100)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 100)
        self.vel_publisher = self.create_publisher(Twist, 'vel_raw', 50)
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 100)
        self.mag_publisher = self.create_publisher(MagneticField, 'imu/mag', 100)

        # Timer for publishing data
        self.create_timer(0.1, self.pub_data)

    def cmd_vel_callback(self, msg):
        # Ensure vx is within the range of -1.0 to 1.0
        if not (-1.0 <= msg.linear.x <= 1.0):
            # self.get_logger().warn("Received vx out of range: {}".format(msg.linear.x))
            return

        # Ensure angular is within the range of -1.0 to 1.0
        if not (-1.0 <= msg.angular.z <= 1.0):
            # self.get_logger().warn("Received angular out of range: {}".format(msg.angular.z))
            return

        # Normalize angular to the range of -0.5 to 0.5
        normalized_angular = msg.angular.z * 0.5

        # Log the received and normalized values for debugging
        # self.get_logger().info("Received vx: {}, normalized angular: {}".format(msg.linear.x, normalized_angular))

        vx = msg.linear.x * 1.0
        vy = 0  # Assuming vy is always zero
        angular = normalized_angular  # Use the normalized angular value

        # Set car motion
        self.car.set_car_motion(vx, vy, angular)

    def rgb_light_callback(self, msg):
        # Handle RGB light changes
        if not isinstance(msg, Int32):
            return
        for _ in range(3):
            self.car.set_colorful_effect(msg.data, 6, parm=1)
    
    def buzzer_callback(self, msg):
        # Handle buzzer activation
        if not isinstance(msg, Bool):
            return
        if msg.data:
            self.car.set_beep(1)
        else:
            self.car.set_beep(0)

    def pub_data(self):
        # Publish sensor and state data periodically
        current_time = Clock().now()

        imu = Imu()
        mag = MagneticField()
        twist = Twist()
        state = JointState()

        # IMU data
        ax, ay, az = self.car.get_accelerometer_data()
        gx, gy, gz = self.car.get_gyroscope_data()
        mx, my, mz = self.car.get_magnetometer_data()

        imu.header.stamp = current_time.to_msg()
        imu.header.frame_id = 'imu_link'
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        mag.header.stamp = current_time.to_msg()
        mag.header.frame_id = 'imu_link'
        mag.magnetic_field.x = mx
        mag.magnetic_field.y = my
        mag.magnetic_field.z = mz

        # Car motion data
        vx, vy, angular = self.car.get_motion_data()
        twist.linear.x = vx
        twist.linear.y = vy * 1000  # steer angle
        twist.angular.z = angular

        # JointState for wheels
        state.header.stamp = current_time.to_msg()
        state.header.frame_id = "joint_states"
        state.name = ["back_right_joint", "back_left_joint", "front_left_steer_joint", "front_left_wheel_joint",
                      "front_right_steer_joint", "front_right_wheel_joint"]
        steer_radis = vy * 1000 * pi / 180.0
        state.position = [0.0, 0.0, steer_radis, 0.0, steer_radis, 0.0]

        # Publish data
        self.vel_publisher.publish(twist)
        self.imu_publisher.publish(imu)
        self.mag_publisher.publish(mag)
        self.joint_state_publisher.publish(state)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)

    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()











