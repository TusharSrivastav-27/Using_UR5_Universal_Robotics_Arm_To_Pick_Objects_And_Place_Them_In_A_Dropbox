#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._planning_group2 = "gripper_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._group2 = moveit_commander.MoveGroupCommander(self._planning_group2)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group2.set_named_target(arg_pose_name)
        plan = self._group2.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()
    
    # FOR FIRST OBJECT
    lst_joint_angles_1 = [math.radians(-7.36),
                          math.radians(-61.03),
                          math.radians(98.54),
                          math.radians(-127.11),
                          math.radians(-92.53),
                          math.radians(135.05)]

    lst_joint_angles_2 = [math.radians(-12.00),
                          math.radians(-57.93),
                          math.radians(110.57),
                          math.radians(-142.06),
                          math.radians(-91.92),
                          math.radians(127.52)]

    lst_joint_angles_3 = [math.radians(-279.38),
                          math.radians(-1.26),
                          math.radians(-61.78),
                          math.radians(331.43),
                          math.radians(-88.76),
                          math.radians(170.91)]
                          
    
    second_joint_angles_1 = [math.radians(-344.16),
                             math.radians(283.30),
                             math.radians(113.89),
                             math.radians(233.28),
                             math.radians(266.39),
                             math.radians(-301.07)]

    second_joint_angles_2 = [math.radians(-345.07),
                          math.radians(297.17),
                          math.radians(119.87),
                          math.radians(213.38),
                          math.radians(266.38),
                          math.radians(-301.99)]

    # FOR THIRD OBJECT
    third_joint_angles_1 = [math.radians(-32.71),
                          math.radians(-65.99),
                          math.radians(96.98),
                          math.radians(-120.72),
                          math.radians(-91.53),
                          math.radians(-297.58)]
                          
    third_joint_angles_2 = [math.radians(-32.94),
                          math.radians(-58.59),
                          math.radians(101.52),
                          math.radians(-132.75),
                          math.radians(-91.54),
                          math.radians(-297.73)]
    
    third_joint_angles_3 = [math.radians(264.77),
                          math.radians(-62.48),
                          math.radians(71.07),
                          math.radians(260.11),
                          math.radians(269.13),
                          math.radians(-0.03)]

    while not rospy.is_shutdown():
        # FOR FIRST OBJECT
        ur5.set_joint_angles(lst_joint_angles_1)
        ur5.set_joint_angles(lst_joint_angles_2)
        ur5.go_to_predefined_pose("gripperP2")
        ur5.set_joint_angles(lst_joint_angles_1)      
        ur5.set_joint_angles(lst_joint_angles_3)
        ur5.go_to_predefined_pose("gripperZeros")
        
        rospy.sleep(1)
        # FOR SECOND OBJECT
        ur5.set_joint_angles(second_joint_angles_1)
        ur5.set_joint_angles(second_joint_angles_2)
        ur5.go_to_predefined_pose("gripperP4")
        ur5.set_joint_angles(second_joint_angles_1)      
        ur5.set_joint_angles(third_joint_angles_3)
        ur5.go_to_predefined_pose("gripperZeros")
        
        # FOR THIRD OBJECT
        ur5.set_joint_angles(third_joint_angles_1)
        ur5.set_joint_angles(third_joint_angles_2)
        ur5.go_to_predefined_pose("gripperP3")
        ur5.set_joint_angles(third_joint_angles_1)      
        ur5.set_joint_angles(third_joint_angles_3)
        ur5.go_to_predefined_pose("gripperZeros")
        
        

    del ur5


if __name__ == '__main__':
    main()
