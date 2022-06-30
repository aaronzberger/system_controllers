#!/usr/bin/python3

import os

import ikpy
import ikpy.chain
import numpy as np
import rospkg

ee_chain = ikpy.chain.Chain.from_urdf_file(os.path.join(
    rospkg.RosPack().get_path('system_controllers'), 'ur5.urdf'))


def ik(goal, current):
    '''
    Method for solving simple inverse kinematic problems.
    This was developed for top down graspig, therefore the solution will be one where the gripper is
    vertical. This might need adjustment for other gripper models.
    Arguments:
        ee_position: List of XYZ-coordinates of the end-effector (ee_link for UR5 setup).
    Returns:
        joint_angles: List of joint angles that will achieve the desired ee position.
    '''

    try:
        assert (len(goal) == 3), 'Invalid EE target! Please specify XYZ-coordinates in a list of length 3.'
        # We want to be able to spedify the ee position in world coordinates, so subtract the position of the
        # base link. This is because the inverse kinematics solver chain starts at the base link.
        ee_position_base = goal - current
        # ee_position_base = (
        #     goal - self.data.body_xpos[self.model.body_name2id('base_link')]
        # )

        # By adding the appr. distance between ee_link and grasp center, we can now specify a world target position
        # for the grasp center instead of the ee_link
        gripper_center_position = ee_position_base + [0, -0.005, 0.16]

        joint_angles = ee_chain.inverse_kinematics(
            gripper_center_position, [0, 0, -1], orientation_mode='X')

        prediction = (
            ee_chain.forward_kinematics(joint_angles)[:3, 3]
            + current
            - [0, -0.005, 0.16])

        diff = abs(prediction - goal)
        error = np.sqrt(diff.dot(diff))
        if error <= 0.05:
            return joint_angles[1:-2]

        print('Failed to find IK solution, error is {}>0.05'.format(error))
        return None
    except Exception as e:
        print(e)
        print('Could not find an inverse kinematics solution.')


def ik_direct(goal_pos, goal_ori):
    '''
    Arguments:
        pos: List of XYZ-coordinates of the end-effector
        ori: List of Rx,Ry,Rz rotations for the end-effector
    Returns:
        joint_angles: List of joint angles that will achieve the desired ee position.
    '''
    try:
        assert (len(goal_pos) == 3 and len(goal_ori) == 3), 'Invalid arguments.'

        # ee_position_base = goal_pos - current
        # gripper_center_position = ee_position_base + [0, -0.005, 0.16]

        joint_angles = ee_chain.inverse_kinematics(
            goal_pos, goal_ori, orientation_mode='X')

        return joint_angles[1:-1]
    except Exception as e:
        print(e)
        print('Could not find an inverse kinematics solution.')
