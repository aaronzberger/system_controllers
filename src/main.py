#!/usr/bin/python3

from ur5a import UR5A_Arm, UR5A_Rail
from ur5b import UR5B_Arm, UR5B_Rail
from free_flyer import Free_Flyer, Free_Flyer_Arm

from termcolor import colored

import rospy


if __name__ == '__main__':
    rospy.init_node('system_controllers', log_level=rospy.INFO)

    subsystems = [UR5A_Arm, UR5A_Rail, UR5B_Arm, UR5B_Rail, Free_Flyer, Free_Flyer_Arm]
    subsystem_names = ['ur5a_arm', 'ur5a_rail', 'ur5b_arm', 'ur5b_rail', 'free_flyer', 'free_flyer_arm']

    for i in range(len(subsystems)):
        print(colored('Creating subsystem {} ({}/{})... '.format(
            subsystems[i].__name__, i + 1, len(subsystems)), color='yellow', attrs=['blink']), end='', flush=True)
        globals()[subsystem_names[i]] = subsystems[i]()
        print('\r' + colored('Waiting for {} server ({}/{})... '.format(
            subsystems[i].__name__, i + 1, len(subsystems)), color='yellow'), end='', flush=True)
        print(colored('CONNECTED', color='green'))

    # ur5a_arm = UR5A_Arm()
    # ur5a_rail = UR5A_Rail()

    # ur5b_arm = UR5B_Arm()
    # ur5b_rail = UR5B_Rail()

    # free_flyer = Free_Flyer()
    # free_flyer_arm = Free_Flyer_Arm()

    rospy.spin()
