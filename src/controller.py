#!/usr/bin/python3

import actionlib
import plan2sim.msg as action
import json
import rospkg
import os
import rospy
from std_msgs.msg import Float64MultiArray
from mj_controller.srv import RegisterGroup


VELOCITY_TOPIC_PREFIX = '/velocities_'


class Controller():
    positions = json.load(os.join(rospkg.RosPack().get_path('system_controllers'), 'src', 'positions.json'))
    pid_constants = json.load(os.join(rospkg.RosPack().get_path('system_controllers'), 'src', 'positions.json'))

    def __init__(self, ns: str, num_motors: int):
        self.ns = ns
        self.num_motors = num_motors

        self._feedback = action.PerformTaskFeedback()
        self._result = action.PerformTaskResult()

        self.action_server = actionlib.SimpleActionServer(ns, action.PerformTaskAction, self.execute, False)
        self.action_server.start()

        velocity_topic = VELOCITY_TOPIC_PREFIX.join(ns)

        rospy.wait_for_service('register_motor_group', timeout=rospy.Duration(secs=20))
        rospy.ServiceProxy('register_motor_group', RegisterGroup)(
            group=ns, num_motors=num_motors, velocity_topic=velocity_topic)

        self.pub_velocities = rospy.Publisher(velocity_topic, Float64MultiArray, queue_size=1)
        self.sub_positions = rospy.Subscriber('/actuator_positions', Float64MultiArray, self.update_positions)

        self.actuator_positions = [0] * num_motors

        self.pid_constants = Controller.pid_constants[ns].values()

    def update_positions(self, _):
        return NotImplementedError('Position updating for server {} not implemented'.format(self.ns))

    def execute(self, _):
        return NotImplementedError('Execution for server {} not implemented'.format(self.ns))
