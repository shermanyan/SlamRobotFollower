#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


class TurtlebotController:

    def __init__(self):
        rospy.init_node("turtlebot_controller", anonymous=True)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # Set rate to 10 Hz
        rospy.sleep(1)  # Add delay to ensure publisher is set up properly

    def publish_twist(self, move_cmd, duration):
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time:
            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()

    def stop_turtlebot(self):
        move_cmd = Twist()
        ######### Your code starts here #########
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        ######### Your code ends here #########
        self.publish_twist(move_cmd, 1)  # Stop for 1 second

    def move_forward(self):
        print("Moving forward...")
        move_cmd = Twist()
        ######### Your code starts here #########
        move_cmd.linear.x = 0.05  # Adjust speed as needed
        move_cmd.angular.z = 0.0
        ######### Your code ends here #########
        self.publish_twist(move_cmd, 10)  # Move forward for 10 seconds

    def move_backward(self):
        print("Moving backward...")
        move_cmd = Twist()
        ######### Your code starts here #########
        move_cmd.linear.x = -0.05  # Adjust speed as needed
        move_cmd.angular.z = 0.0
        ######### Your code ends here #########
        self.publish_twist(move_cmd, 10)  # Move backward for 10 seconds

    def turn_left(self):
        print("Turning left...")
        move_cmd = Twist()
        ######### Your code starts here #########
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.2  # Adjust angular speed as needed
        ######### Your code ends here #########
        self.publish_twist(move_cmd, 5)  # Turn left for 5 seconds

    def turn_right(self):
        print("Turning right...")
        move_cmd = Twist()
        ######### Your code starts here #########
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = -0.2  # Adjust angular speed as needed
        ######### Your code ends here #########
        self.publish_twist(move_cmd, 5)  # Turn right for 5 seconds

    def move_sequence(self):
        # Move forward
        self.move_forward()

        # Turn left
        self.turn_left()

        # Turn right
        self.turn_right()

        # Move backward
        self.move_backward()

        # Stop at the end of sequence
        self.stop_turtlebot()


def main():
    controller = TurtlebotController()
    try:
        controller.move_sequence()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
