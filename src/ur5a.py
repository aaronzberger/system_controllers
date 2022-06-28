#!/usr/bin/python3

import rospy
from controller import Controller
from std_msgs.msg import Float64MultiArray
from simple_pid import PID

ACTION_HZ = 25

# TODO: Should be defined per action
TOLERANCE = 0.01


class UR5A_Arm(Controller):
    def __init__(self):
        super().__init__('ur5a_arm', 6)

        self.pid_list = []
        for motor in self.pid_constants:
            self.pid_list.append(PID(
                motor['PID'], setpoint=motor['setpoint'],
                output_limits=(-motor['max_vel'], motor['max_vel']),
                sample_time=None))

    def update_positions(self, data: Float64MultiArray):
        assert len(self.actuator_positions) == len(data.data), \
            'Received {} actuator positions but the group {} expected {}'.format(
                len(data.data), self.ns, len(self.actuator_positions))
        self.actuator_positions = data.data

    def execute(self, goal):
        # TODO: Interpret the text position into a 3d position using positions.json

        # TODO: Run IK

        # TODO: Set IK positions to setpoints of the PIDS

        finished = False
        max_steps = (goal.end_time - goal.start_time) * ACTION_HZ

        steps = 0
        while not finished:
            # Publish the velocities
            self.pub_velocities(Float64MultiArray(
                data=[self.pid_list[i](self.actuator_positions[i]) for i in range(self.num_motors)]))
            # TODO: May be able to remove Float64MultiArray above, (unsure?)

            # TODO: Calculate percentage complete based on position and setpoint
            self._feedback.percent_complete += 10
            self.action_server.publish_feedback(self._feedback)

            rospy.Rate(ACTION_HZ).sleep()

            if self.action_server.is_preempt_requested():
                break

            steps += 1

            deltas = [abs(self.pid_list[i].setpoint - self.actuator_positions[i]) for i in range(self.num_motors)]
            if max(deltas) < TOLERANCE:
                finished = True

            if steps >= max_steps:
                break

        self._result.result.name = goal.name
        self._result.result.time_ended = rospy.Time().now()

        if not finished:
            # Report a failure
            self._result.result.final_position = 'unknown'
            self.action_server.set_aborted(self._result)
        else:
            # Report a success
            self._result.result.final_position = goal.end
            self.action_server.set_succeeded(self._result)


class UR5A_Rail(Controller):
    def __init__(self):
        super().__init__('ur5a_rail', 1)

        self.pid_list = []
        for motor in self.pid_constants:
            self.pid_list.append(PID(
                motor['PID'], setpoint=motor['setpoint'],
                output_limits=(-motor['max_vel'], motor['max_vel']),
                sample_time=None))

    def update_positions(self, data: Float64MultiArray):
        assert len(self.actuator_positions) == len(data.data), \
            'Received {} actuator positions but the group {} expected {}'.format(
                len(data.data), self.ns, len(self.actuator_positions))
        self.actuator_positions = data.data

    def execute(self, goal):
        # TODO: Interpret the text position into a 3d position using positions.json and set this as the PID setpoint

        finished = False
        max_steps = (goal.end_time - goal.start_time) * ACTION_HZ

        steps = 0
        while not finished:
            # Publish the velocities
            self.pub_velocities(Float64MultiArray(
                data=[self.pid_list[i](self.actuator_positions[i]) for i in range(self.num_motors)]))
            # TODO: May be able to remove Float64MultiArray above, (unsure?)

            # TODO: Calculate percentage complete based on position and setpoint
            self._feedback.percent_complete += 10
            self.action_server.publish_feedback(self._feedback)

            rospy.Rate(ACTION_HZ).sleep()

            if self.action_server.is_preempt_requested():
                break

            steps += 1

            deltas = [abs(self.pid_list[i].setpoint - self.actuator_positions[i]) for i in range(self.num_motors)]
            if max(deltas) < TOLERANCE:
                finished = True

            if steps >= max_steps:
                break

        self._result.result.name = goal.name
        self._result.result.time_ended = rospy.Time().now()

        if not finished:
            # Report a failure
            self._result.result.final_position = 'unknown'
            self.action_server.set_aborted(self._result)
        else:
            # Report a success
            self._result.result.final_position = goal.end
            self.action_server.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('ur5a', log_level=rospy.INFO)

    ur5a_arm = UR5A_Arm()
    ur5a_rail = UR5A_Rail()

    rospy.spin()
