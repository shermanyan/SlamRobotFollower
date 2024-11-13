#!/usr/bin/env python3
import rospy
from time import sleep, time
from turtlebot3_msgs.msg import SensorState


class Cliff:
    """
    Prints the measured value from the attached DMS-80 Distance Sensor.

    The DMS-80 should be connected to the 'ROBOTIS_5-PIN 2' pin (shown here:
    https://emanual.robotis.com/docs/en/parts/controller/opencr10/#robotis-5-pin-connector). The 'cliff' value of the
    SensorState object holds the measurement. See this link for more information:
    https://emanual.robotis.com/docs/en/platform/turtlebot3/additional_sensors/#ir
    """

    def __init__(self, print_dt: float = 0.5):
        self.cliff_sub = rospy.Subscriber("/sensor_state", SensorState, self.sensor_state_callback, queue_size=1)
        self.print_dt = print_dt
        self.t_last_print = time()

    def sensor_state_callback(self, state: SensorState):
        if self.t_last_print + self.print_dt > time():
            return
        self.t_last_print = time()
        raw = state.cliff
        distance = 0.0  # TODO: fit function mapping raw to distance

        ######### Your code starts here #########
        # calculation from raw sensor value to distance (Step 3.3 of lab)
        
        ######### Your code ends here #########

        print(f"raw: {raw}\tdistance: {distance}")


if __name__ == "__main__":
    rospy.init_node("turtlebot3_cliff")
    try:
        cliff = Cliff()
        while not rospy.is_shutdown():
            sleep(0.1)
    except rospy.ROSInterruptException:
        pass
