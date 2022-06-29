#!/usr/bin/python3

import actionlib
import plan2sim.msg as action
import json
import rospkg
import os
import rospy
from std_msgs.msg import Float64MultiArray
from mj_controller.srv import RegisterGroup
import numpy as np


VELOCITY_TOPIC_PREFIX = '/velocities_'


class Controller():
    positions = json.load(open(os.path.join(
        rospkg.RosPack().get_path('system_controllers'), 'src', 'positions.json'), 'r'))
    pid_constants = json.load(open(os.path.join(
        rospkg.RosPack().get_path('system_controllers'), 'src', 'pid_constants.json'), 'r'))

    def __init__(self, ns: str, num_motors: int):
        self.ns = ns
        self.num_motors = num_motors

        self._feedback = action.PerformTaskFeedback()
        self._result = action.PerformTaskResult()

        velocity_topic = VELOCITY_TOPIC_PREFIX.join(ns)

        rospy.wait_for_service('register_motor_group', timeout=rospy.Duration(secs=20))
        registration_response = rospy.ServiceProxy('register_motor_group', RegisterGroup)(
            group=ns, num_motors=num_motors, velocity_topic=velocity_topic)
        self.motor_ids = registration_response.ids

        self.pub_velocities = rospy.Publisher(velocity_topic, Float64MultiArray, queue_size=1)
        self.sub_positions = rospy.Subscriber(
            registration_response.position_topic, Float64MultiArray, self.update_positions)

        self.actuator_positions = np.zeros(num_motors)

        self.pid_constants = Controller.pid_constants[ns].values()

        self.action_server = actionlib.SimpleActionServer(ns, action.PerformTaskAction, self.execute, False)
        self.action_server.start()

    def update_positions(self, data: Float64MultiArray):
        received_positions = np.take(data.data, self.motor_ids)
        assert self.actuator_positions.size == received_positions.size, \
            'Received {} actuator positions but the group {} expected {}.'.format(
                received_positions.size, self.ns, self.actuator_positions.size) + \
            'You may need to change the number of motors in this subsystem\'s creation'
        self.actuator_positions = received_positions

    def execute(self, _):
        return NotImplementedError('Execution for server {} not implemented'.format(self.ns))
