#!/usr/bin/env python3
from math import inf
import queue
from time import sleep, time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# P controller class
class PController:
    """
    Generates control action taking into account instantaneous error (proportional action).
    """

    def __init__(self, kP, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        # Initialize variables here
        ######### Your code starts here #########
        self.kP = kP
        self.u_min = u_min
        self.u_max = u_max
        ######### Your code ends here #########

    def control(self, err, t):
        dt = t - self.t_prev
        if dt <= 1e-6:
            return 0

        # Compute control action here
        ######### Your code starts here #########
        def return_clamped(val):
            return max(self.u_min, min(val, self.u_max))

        u = (self.kP * err)
        return return_clamped(u)
        ######### Your code ends here #########

# PD controller class
class PDController:
    """
    Generates control action taking into account instantaneous error (proportional action)
    and rate of change of error (derivative action).
    """

    def __init__(self, kP, kD, kS, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        # Initialize variables here
        ######### Your code starts here #########
        self.kP = kP
        self.kD = kD
        self.kS = kS
        self.err_dif = 0
        self.err_prev = 0
        self.err_hist = queue.Queue(self.kS)
        self.t_prev = 0
        self.u_min = u_min
        self.u_max = u_max
        ######### Your code ends here #########

    def control(self, err, t):
        dt = t - self.t_prev
        if dt <= 1e-6:
            return 0

        # Compute control action here
        ######### Your code starts here #########
        def return_clamped(val):
            return max(self.u_min, min(val, self.u_max))

        if self.err_hist.full():
            self.err_hist.get()
        self.err_hist.put(err)
        self.err_dif = err - self.err_prev
        u = (self.kP * err) + (self.kD * self.err_dif / dt)
        self.err_prev = err
        self.t_prev = t
        return return_clamped(u)
        ######### Your code ends here #########


class RobotController:
    def __init__(self, desired_distance: float):
        print("\nMake the robot follow the wall on its left by maintaining the distance from it using LIDAR.\n")

        # ROS1 infrastructure
        rospy.init_node("robot_controller", anonymous=True)
        self.laserscan_sub = rospy.Subscriber("/scan", LaserScan, self.robot_laserscan_callback)
        self.robot_ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Define PD controller for lateral control here
        ######### Your code starts here #########
        self.pd_lat = PDController(0.5, 0.1, 10, -2.84, 2.84)
        ######### Your code ends here #########

        self.desired_distance = desired_distance  # Desired distance from the wall
        self.ir_distance = None

    def robot_laserscan_callback(self, lscan: LaserScan):
        left = lscan.ranges[80:100]
        left = [x for x in left if x != inf]
        if len(left) > 0:
            self.ir_distance = sum(left) / len(left)

    def control_loop(self):

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if self.ir_distance is None:
                print("Waiting for IR sensor readings")
                sleep(0.1)
                continue

            ctrl_msg = Twist()

            ######### Your code starts here #########
            # cte = self.desired_distance - self.ir_distance # <-- NOTE: this way is incorrect
            cte = self.ir_distance - self.desired_distance  # correct
            tstamp = time()
            u = self.pd_lat.control(cte, tstamp)
            ctrl_msg.linear.x = 0.2
            ctrl_msg.angular.z = u
            ######### Your code ends here #########

            self.robot_ctrl_pub.publish(ctrl_msg)
            print(f"dist: {round(self.ir_distance, 4)}\ttgt: {round(self.desired_distance, 4)}\tu: {round(u, 4)}")
            rate.sleep()


if __name__ == "__main__":
    desired_distance = 0.5
    controller = RobotController(desired_distance)
    try:
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
