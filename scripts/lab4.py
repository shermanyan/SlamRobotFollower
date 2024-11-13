#!/usr/bin/env python3
import queue
from time import sleep, time

import rospy
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState


# P controller class
class PController:
    """
    Generates control action taking into account instantaneous error (proportional action).
    """

    def __init__(self, kP, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        # Initialize variables here
        ######### Your code starts here #########

        ######### Your code ends here #########

    def control(self, err, t):
        dt = t - self.t_prev
        if dt <= 1e-6:
            return 0

        # Compute control action here
        ######### Your code starts here #########

        ######### Your code ends here #########

# PD controller class
class PDController:
    """
    Generates control action taking into account instantaneous error (proportional action)
    and rate of change of error (derivative action).
    """

    def __init__(self, kP, kD, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        # Initialize variables here
        ######### Your code starts here #########

        ######### Your code ends here #########

    def control(self, err, t):
        dt = t - self.t_prev
        if dt <= 1e-6:
            return 0

        # Compute control action here
        ######### Your code starts here #########

        ######### Your code ends here #########


class RobotController:
    def __init__(self, desired_distance: float):
        print("\nMake the robot follow the wall on its left by maintaining the distance from it using a sonar sensor.\n")

        # ROS1 infrastructure
        rospy.init_node("robot_controller", anonymous=True)
        self.cliff_sub = rospy.Subscriber("/sensor_state", SensorState, self.sensor_state_callback, queue_size=1)
        self.robot_ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Define PD controller for wall following here
        ######### Your code starts here #########

        ######### Your code ends here #########

        self.desired_distance = desired_distance  # Desired distance from the wall
        self.ir_distance = None

    def sensor_state_callback(self, state: SensorState):
        raw = state.cliff
        ######### Your code starts here #########
        # conversion from raw sensor values to distance. Use equation from Lab 2

        ######### Your code ends here #########
        # print(f"raw: {raw}\tdistance: {distance}")
        self.ir_distance = distance

    def control_loop(self):

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if self.ir_distance is None:
                print("Waiting for IR sensor readings")
                sleep(0.1)
                continue

            ctrl_msg = Twist()

            # using PD controller, compute and send motor commands
            ######### Your code starts here #########

            ######### Your code ends here #########

            self.robot_ctrl_pub.publish(ctrl_msg)
            print(f"dist: {round(self.ir_distance, 4)}\ttgt: {round(self.desired_distance, 4)}\tu: {round(u, 4)}")
            rate.sleep()


if __name__ == "__main__":
    desired_distance = 0.4
    controller = RobotController(desired_distance)
    try:
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
