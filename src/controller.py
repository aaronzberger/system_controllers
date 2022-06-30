#!/usr/bin/python3

import json
import os

import actionlib
import numpy as np
import plan2sim.msg as action
import rospkg
import rospy
from mj_controller.srv import RegisterGroup
from simple_pid import PID
from std_msgs.msg import Float64MultiArray

VELOCITY_TOPIC_PREFIX = '/velocities_'

ACTION_HZ = 25

# TODO: Should be defined per action
TOLERANCE = 0.01


class Controller():
    defined_positions = json.load(open(os.path.join(
        rospkg.RosPack().get_path('system_controllers'), 'src', 'positions.json'), 'r'))
    pid_constants = json.load(open(os.path.join(
        rospkg.RosPack().get_path('system_controllers'), 'src', 'pid_constants.json'), 'r'))

    def __init__(self, ns: str, num_motors: int):
        self.ns = ns
        self.num_motors = num_motors

        self._feedback = action.PerformTaskFeedback()
        self._result = action.PerformTaskResult()

        self.defined_positions = Controller.defined_positions[ns]
        self.pid_constants = Controller.pid_constants[ns].values()

        self.actuator_positions = np.zeros(num_motors)

        self._pid_list = []
        for motor in self.pid_constants:
            self._pid_list.append(PID(
                *motor['PID'], setpoint=motor['setpoint'],
                output_limits=(-motor['max_vel'], motor['max_vel']),
                sample_time=None))

        velocity_topic = VELOCITY_TOPIC_PREFIX.join(ns)

        rospy.wait_for_service('register_motor_group', timeout=rospy.Duration(secs=20))
        registration_response = rospy.ServiceProxy('register_motor_group', RegisterGroup)(
            group=ns, num_motors=num_motors, velocity_topic=velocity_topic)
        self.motor_ids = registration_response.ids

        self._pub_velocities = rospy.Publisher(velocity_topic, Float64MultiArray, queue_size=1)
        self._sub_positions = rospy.Subscriber(
            registration_response.position_topic, Float64MultiArray, self.update_positions)

        self._action_server = actionlib.SimpleActionServer(ns, action.PerformTaskAction, self.execute, False)
        self._action_server.start()

    def update_positions(self, data: Float64MultiArray):
        received_positions = np.take(data.data, self.motor_ids)
        assert self.actuator_positions.size == received_positions.size, \
            'Received {} actuator positions but the group {} expected {}.'.format(
                received_positions.size, self.ns, self.actuator_positions.size) + \
            'You may need to change the number of motors in this subsystem\'s creation'
        self.actuator_positions = received_positions

    def position_to_joint_angles(self, positions):
        '''This function must be implemented by any subsystem where the joint angles
        required to reach the desired position is not exactly the position'''
        return positions

    def execute(self, goal):
        self._result.result.name = goal.task.name
        start_time = rospy.Time().now()

        def fail():
            self._result.result.time_ended = (rospy.Time().now() - start_time).to_sec() + goal.task.start_time
            self._result.result.final_position = 'unknown'
            self._result.result.succeeded = False
            self._action_server.set_aborted(self._result)

        # Interpret the text position into a 3d position
        try:
            positions = self.defined_positions[goal.task.end]
        except KeyError:
            print('Unknown position \'{}\' for system \'{}\''.format(goal.task.end, self.ns))
            return fail()

        # Translate these positions into joint angles according to this subsystem
        joint_angles = self.position_to_joint_angles(positions)

        assert len(joint_angles) == self.num_motors, \
            'Number of motors mismatch for position \'{}\''.format(goal.task.end)

        # Set the desired positions in the controllers
        for setpt, controller in zip(joint_angles, self._pid_list):
            controller.setpoint = setpt

        max_steps = (goal.task.end_time - goal.task.start_time) * ACTION_HZ
        steps = 0

        # Calculate the offset of each joint to its target, to be used to determine the success of this action
        deltas = [abs(self._pid_list[i].setpoint - self.actuator_positions[i]) for i in range(self.num_motors)]
        while max(deltas) > TOLERANCE:
            # Publish the velocities
            self._pub_velocities.publish(Float64MultiArray(
                data=[self._pid_list[i](self.actuator_positions[i]) for i in range(self.num_motors)]))
            # TODO: May be able to remove Float64MultiArray above, (unsure?)

            # TODO: Calculate percentage complete based on position and setpoint
            self._feedback.percent_complete += 1
            self._action_server.publish_feedback(self._feedback)

            rospy.Rate(ACTION_HZ).sleep()

            if self._action_server.is_preempt_requested():
                return fail()

            if steps >= max_steps:
                return fail()

            steps += 1

            deltas = [abs(self._pid_list[i].setpoint - self.actuator_positions[i]) for i in range(self.num_motors)]

        self._result.result.time_ended = (rospy.Time().now() - start_time).to_sec() + goal.start_time
        self._result.result.final_position = goal.task.end
        self._result.result.succeeded = True
        self._action_server.set_succeeded(self._result)
