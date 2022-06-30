#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64MultiArray

from controller import ACTION_HZ, TOLERANCE, Controller


class Free_Flyer(Controller):
    def __init__(self):
        super().__init__('free_flyer', 3)

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

        def go_to_position(position, max_steps, tolerance, fail_ok=False):
            assert len(position) == self.num_motors, \
                'Number of motors mismatch for position \'{}\''.format(goal.task.end)

            # Set the desired positions in the controllers
            for setpt, controller in zip(position, self._pid_list):
                controller.setpoint = setpt

            steps = 0

            # Calculate the offset of each joint to its target, to be used to determine the success of this action
            deltas = [abs(self._pid_list[i].setpoint - self.actuator_positions[i]) for i in range(self.num_motors)]
            while max(deltas) > tolerance:
                # Publish the velocities
                self._pub_velocities.publish(Float64MultiArray(
                    data=[self._pid_list[i](self.actuator_positions[i]) for i in range(self.num_motors)]))
                # TODO: May be able to remove Float64MultiArray above, (unsure?)

                # TODO: Calculate percentage complete based on position and setpoint
                self._feedback.percent_complete += 1
                self._action_server.publish_feedback(self._feedback)

                rospy.Rate(ACTION_HZ).sleep()

                if self._action_server.is_preempt_requested():
                    return None if fail_ok else fail()

                if steps >= max_steps:
                    return None if fail_ok else fail()

                steps += 1

                deltas = [abs(self._pid_list[i].setpoint - self.actuator_positions[i]) for i in range(self.num_motors)]

        first_position = [joint_angles[0], *self.actuator_positions[1:]]
        max_total_steps = (goal.task.end_time - goal.task.start_time) * ACTION_HZ
        go_to_position(first_position, max_steps=max_total_steps / 3, tolerance=0.1, fail_ok=True)
        go_to_position(joint_angles, max_steps=2/3 * max_total_steps, tolerance=TOLERANCE)

        self._result.result.time_ended = (rospy.Time().now() - start_time).to_sec() + goal.start_time
        self._result.result.final_position = goal.task.end
        self._result.result.succeeded = True
        self._action_server.set_succeeded(self._result)


class Free_Flyer_Arm(Controller):
    def __init__(self):
        super().__init__('free_flyer_arm', 1)
