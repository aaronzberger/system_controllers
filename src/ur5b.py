#!/usr/bin/python3

from controller import Controller
from ik import ik_direct


class UR5B_Arm(Controller):
    def __init__(self):
        super().__init__('ur5b_arm', 6)

    def position_to_joint_angles(self, positions):
        # current = self.sim.data.body_xpos[self.model.body_name2id('base_link')]
        # Run IK
        joint_angles = ik_direct(*positions)
        return joint_angles


class UR5B_Rail(Controller):
    def __init__(self):
        super().__init__('ur5b_rail', 1)


class UR5B_Gripper(Controller):
    def __init__(self):
        super().__init__('ur5b_gripper', 1)
