#!/usr/bin/python3

from controller import Controller
from ik import ik_direct


class UR5A_Arm(Controller):
    def __init__(self):
        super().__init__('ur5a_arm', 7)

    def position_to_joint_angles(self, positions):
        # current = self.sim.data.body_xpos[self.model.body_name2id('base_link')]
        # Run IK
        joint_angles = ik_direct(*positions)
        return joint_angles


class UR5A_Rail(Controller):
    def __init__(self):
        super().__init__('ur5a_rail', 1)
