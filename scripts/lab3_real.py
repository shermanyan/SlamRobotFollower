#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Twist
from turtlebot3_msgs.msg import SensorState
import tf
import math


class OdometryPublisher:
    def __init__(self):
        rospy.init_node("odometry_publisher", anonymous=True)

        # Publisher to the /custom_odom topic
        self.odom_pub = rospy.Publisher("/custom_odom", Odometry, queue_size=10)

        # Subscriber to the /sensor_state topic
        self.sensor_sub = rospy.Subscriber("/sensor_state", SensorState, self.sensor_callback)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.left_encoder = 0
        self.right_encoder = 0
        self.last_left_encoder = None
        self.last_right_encoder = None

        ######### Your code starts here #########
        # TurtleBot3 Burger parameters from manual (https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
        self.TICK_TO_RAD =
        self.wheel_radius =
        self.wheel_separation =
        ######### Your code ends here #########

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

    def sensor_callback(self, msg):
        self.left_encoder = msg.left_encoder
        self.right_encoder = msg.right_encoder

        # Initialize last encoder values when the first message is received
        if self.last_left_encoder is None or self.last_right_encoder is None:
            self.last_left_encoder = self.left_encoder
            self.last_right_encoder = self.right_encoder

    def update_odometry(self):
        # Ensure encoder values are initialized
        if self.last_left_encoder is None or self.last_right_encoder is None:
            return

        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()

        ######### Your code starts here #########
        # add odometry equations to calculate robot's self.x, self.y, self.theta given encoder values

        ######### Your code ends here #########

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist()
        odom.twist.twist.linear.x = delta_s / dt
        odom.twist.twist.angular.z = delta_theta / dt

        self.odom_pub.publish(odom)
        self.last_time = self.current_time

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.update_odometry()
            rate.sleep()


if __name__ == "__main__":
    try:
        print("Publishing odometry under /custom_odom...")
        odom_pub = OdometryPublisher()
        odom_pub.run()
    except rospy.ROSInterruptException:
        pass
