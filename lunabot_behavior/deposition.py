import rospy

from lunabot_msgs.msg import RobotEffort, RobotSensors

import time
import interrupts

'''
State to deposit collected regolith onto the berm by spinning the auger
'''
class Deposition:
    def sensors_callback(self, msg: RobotSensors):
        self.robot_sensors = msg

    def __init__(self, effort_publisher: rospy.Publisher = None):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """

        if effort_publisher is None:
            self.effort_publisher = rospy.Publisher("/effort", RobotEffort, queue_size=1, latch=True)
            rospy.init_node('deposition_node')
        else:
            self.effort_publisher = effort_publisher

        self.robot_sensors = RobotSensors()

        rospy.Subscriber("/sensors", RobotSensors, self.sensors_callback)

        self.rate = rospy.Rate(10)  # 10hz

        self.deposition_power = 127 # TODO change if needed

        self.load_cell_threshold = 1 # in kilograms, TODO test / verify

    def deposit(self):
        """
        Spin the auger until the load cell detects that the bin is empty
        """

        time.sleep(0.1)

        effort_message = RobotEffort()
        effort_message.deposit = self.deposition_power

        while True:
            self.effort_publisher.publish(effort_message)

            weight = self.robot_sensors.load_cell_weights[0] + self.robot_sensors.load_cell_weights[1]

            if weight < self.load_cell_threshold:
                break

            if (interrupts.check_for_interrupts() != interrupts.Errors.FINE):
                return False

            self.rate.sleep()

        effort_message.deposit = 0
        self.effort_publisher.publish(effort_message)

        return True