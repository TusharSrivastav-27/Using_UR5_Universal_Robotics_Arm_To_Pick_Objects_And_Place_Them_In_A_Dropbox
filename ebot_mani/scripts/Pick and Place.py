#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import threading



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
        
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()
    
    # FOR FIRST OBJECT #1-----------------------------------------------------------------------
    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = 0.581248822715
    ur5_pose_1.position.y = -0.00943461531741
    ur5_pose_1.position.z = 1.05899443087
    ur5_pose_1.orientation.x = -0.282852432617
    ur5_pose_1.orientation.y = -0.64387462585
    ur5_pose_1.orientation.z = 0.648808859768
    ur5_pose_1.orientation.w = 0.2906321232
    lst_joint_angles_1 = [math.radians(-11.7520459631),
                          math.radians(-75.0317352443),
                          math.radians(95.8779956492),
                          math.radians(-110.145746775),
                          math.radians(-90.2579089797),
                          math.radians(126.097393997)]
    t_pose = threading.Thread(target=ur5.go_to_pose, args=(ur5_pose_1,)) 
    t_joint_angle = threading.Thread(target=ur5.set_joint_angles, args=(lst_joint_angles_1,))
    
    t_pose.start() 
    t_joint_angle.start() 

    # 3. Wait for the two threads to complete before moving to the next line.
    t_pose.join() 
    t_joint_angle.join() 
    
    # FOR FIRST OBJECT #2------------------------------------------------------------------------
    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x =  0.568029657331
    ur5_pose_2.position.y = -0.0179713674527
    ur5_pose_2.position.z =  0.834092521937
    ur5_pose_2.orientation.x = -0.306308923878
    ur5_pose_2.orientation.y = -0.632942924203
    ur5_pose_2.orientation.z = 0.63804284791
    ur5_pose_2.orientation.w = 0.313782443876
    lst_joint_angles_2 = [math.radians(-12.8851145818),
                          math.radians(-57.716677319),
                          math.radians(110.710573395),
                          math.radians(-142.310315899),
                          math.radians(-90.2649674332),
                          math.radians(129.130317277)]

    t_pose = threading.Thread(target=ur5.go_to_pose, args=(ur5_pose_2,)) 
    t_joint_angle = threading.Thread(target=ur5.set_joint_angles, args=(lst_joint_angles_2,))
    
    t_pose.start() 
    t_joint_angle.start() 

    # 3. Wait for the two threads to complete before moving to the next line.
    t_pose.join() 
    t_joint_angle.join()
    ur5.go_to_predefined_pose("gripperP2") # GRIPPING OBJECT 1 -----------------------------------
    rospy.sleep(1)
    # FOR FIRST OBJECT #3------------------------------------------------------------------------
    ur5_pose_3 = geometry_msgs.msg.Pose()
    ur5_pose_3.position.x = 0.582224745157
    ur5_pose_3.position.y = -0.00921951449693
    ur5_pose_3.position.z = 1.13812410658
    ur5_pose_3.orientation.x = -0.28
    ur5_pose_3.orientation.y = -0.64
    ur5_pose_3.orientation.z = 0.64
    ur5_pose_3.orientation.w = 0.29 
    lst_joint_angles_3 = [math.radians(-11.7110364565),
                          math.radians(-77.7013597603),
                          math.radians(87.2589140005),
                          math.radians(-98.8596333744),
                          math.radians(-90.2550552857),
                          math.radians(126.136602524)]

    t_pose = threading.Thread(target=ur5.go_to_pose, args=(ur5_pose_3,)) 
    t_joint_angle = threading.Thread(target=ur5.set_joint_angles, args=(lst_joint_angles_3,))
    
    t_pose.start() 
    t_joint_angle.start()
    t_pose.join() 
    t_joint_angle.join()

    # FOR FIRST OBJECT #4------------------------------------------------------------------------
    ur5_pose_4 = geometry_msgs.msg.Pose()
    ur5_pose_4.position.x = 0.0208866862506
    ur5_pose_4.position.y = 0.711057687693
    ur5_pose_4.position.z = 1.14383798823
    ur5_pose_4.orientation.x = -0.282836458352
    ur5_pose_4.orientation.y = -0.643833096211
    ur5_pose_4.orientation.z = 0.648881545917
    ur5_pose_4.orientation.w = 0.290577393167
    lst_joint_angles_4 = [math.radians(79.4912814933),
                          math.radians(-59.4238139137),
                          math.radians(61.0029157148),
                          math.radians(-91.3266649567),
                          math.radians(-89.294983189),
                          math.radians(217.331330926)]

    t_pose = threading.Thread(target=ur5.go_to_pose, args=(ur5_pose_4,)) 
    t_joint_angle = threading.Thread(target=ur5.set_joint_angles, args=(lst_joint_angles_4,))
    
    t_pose.start() 
    t_joint_angle.start()
    
    t_pose.join() 
    t_joint_angle.join()
    ur5.go_to_predefined_pose("gripperZeros") # GRIPPING OBJECT 1 ----------------------------------- 
    
    # FOR SECOND OBJECT #1------------------------------------------------------------------------
    ur5_pose_5 = geometry_msgs.msg.Pose()
    ur5_pose_5.position.x = 0.457815691448
    ur5_pose_5.position.y = 0.226580049351
    ur5_pose_5.position.z = 0.999945709374
    ur5_pose_5.orientation.x = -0.225603648801
    ur5_pose_5.orientation.y = 0.672423859654
    ur5_pose_5.orientation.z = -0.670850013889
    ur5_pose_5.orientation.w = 0.216585792426
    lst_joint_angles_5 = [math.radians(13.9935719024),
                          math.radians(-80.3575395988),
                          math.radians(112.207609276),
                          math.radians(-121.111948476),
                          math.radians(-89.9266853209),
                          math.radians(67.5512658176)]

    t_pose = threading.Thread(target=ur5.go_to_pose, args=(ur5_pose_5,)) 
    t_joint_angle = threading.Thread(target=ur5.set_joint_angles, args=(lst_joint_angles_5,))
    
    t_pose.start() 
    t_joint_angle.start()

    # 3. Wait for the two threads to complete before moving to the next line.
    t_pose.join() 
    t_joint_angle.join()

    # FOR SECOND OBJECT #2------------------------------------------------------------------------
    ur5_pose_6 = geometry_msgs.msg.Pose()
    ur5_pose_6.position.x = 0.457868385881
    ur5_pose_6.position.y = 0.227786247201
    ur5_pose_6.position.z = 0.837848984003
    ur5_pose_6.orientation.x = -0.225616700892
    ur5_pose_6.orientation.y = 0.672447936795
    ur5_pose_6.orientation.z = -0.670867268825
    ur5_pose_6.orientation.w = 0.216443951631 
    lst_joint_angles_6 = [math.radians(14.1263655235),
                          math.radians(-63.1650413986),
                          math.radians(120.254649674),
                          math.radians(-146.339107671),
                          math.radians(-89.9252478753),
                          math.radians(67.6950018271)]

    t_pose = threading.Thread(target=ur5.go_to_pose, args=(ur5_pose_6,)) 
    t_joint_angle = threading.Thread(target=ur5.set_joint_angles, args=(lst_joint_angles_6,))
    
    t_pose.start() 
    t_joint_angle.start()
    t_pose.join() 
    t_joint_angle.join()
    ur5.go_to_predefined_pose("gripperP5")

    # FOR SECOND OBJECT #3------------------------------------------------------------------------
    ur5_pose_7 = geometry_msgs.msg.Pose()
    ur5_pose_7.position.x = 0.461355279468
    ur5_pose_7.position.y = 0.228389388581
    ur5_pose_7.position.z = 1.11345939711
    ur5_pose_7.orientation.x = -0.225678301957
    ur5_pose_7.orientation.y =  0.672374393989
    ur5_pose_7.orientation.z = -0.670893456397
    ur5_pose_7.orientation.w = 0.21652701563
    lst_joint_angles_7 = [math.radians(14.0960600299),
                          math.radians(-86.1196270631),
                          math.radians(100.661533978),
                          math.radians(-103.793674994),
                          math.radians(-89.9336603889),
                          math.radians(67.6524184554)]

    t_pose = threading.Thread(target=ur5.go_to_pose, args=(ur5_pose_7,)) 
    t_joint_angle = threading.Thread(target=ur5.set_joint_angles, args=(lst_joint_angles_7,))
    
    t_pose.start() 
    t_joint_angle.start()
    t_pose.join() 
    t_joint_angle.join()

    # 3. Wait for the two threads to complete before moving to the next line.
    t_pose.join() 
    t_joint_angle.join()

     # FOR SECOND OBJECT #4------------------------------------------------------------------------
    ur5_pose_8 = geometry_msgs.msg.Pose()
    ur5_pose_8.position.x = 0.0189333053514
    ur5_pose_8.position.y = -0.714016880696
    ur5_pose_8.position.z = 1.13224395267
    ur5_pose_8.orientation.x = -0.215582367573
    ur5_pose_8.orientation.y = 0.675624660112
    ur5_pose_8.orientation.z = -0.674101589024
    ur5_pose_8.orientation.w = 0.206500869525
    lst_joint_angles_8 = [math.radians(-97.271093151),
                          math.radians(-58.8964505622),
                          math.radians(61.9853311122),
                          math.radians(-93.2834644037),
                          math.radians(-90.720324439),
                          math.radians(-42.0030873682)]

    t_pose = threading.Thread(target=ur5.go_to_pose, args=(ur5_pose_8,)) 
    t_joint_angle = threading.Thread(target=ur5.set_joint_angles, args=(lst_joint_angles_8,))
    
    t_pose.start() 
    t_joint_angle.start()
    t_pose.join() 
    t_joint_angle.join()

    # 3. Wait for the two threads to complete before moving to the next line.
    t_pose.join() 
    t_joint_angle.join()
    ur5.go_to_predefined_pose("gripperZeros")

     # FOR THIRD OBJECT #1------------------------------------------------------------------------
    ur5_pose_9 = geometry_msgs.msg.Pose()
    ur5_pose_9.position.x = 0.53452226287
    ur5_pose_9.position.y = -0.251810890113
    ur5_pose_9.position.z = 1.07313197855
    ur5_pose_9.orientation.x = -0.0183363161334
    ur5_pose_9.orientation.y = -0.705883894241
    ur5_pose_9.orientation.z = 0.707561459353
    ur5_pose_9.orientation.w = 0.0273585197124
    lst_joint_angles_9 = [math.radians(-35.8702234821),
                          math.radians(-74.4267215814),
                          math.radians(92.9400062432),
                          math.radians(-107.983374402),
                          math.radians(-90.5216604773),
                          math.radians(57.8354940054)]

    t_pose = threading.Thread(target=ur5.go_to_pose, args=(ur5_pose_9,)) 
    t_joint_angle = threading.Thread(target=ur5.set_joint_angles, args=(lst_joint_angles_9,))
    
    t_pose.start() 
    t_joint_angle.start()
    t_pose.join() 
    t_joint_angle.join()

    # 3. Wait for the two threads to complete before moving to the next line.
    t_pose.join() 
    t_joint_angle.join()
    

     # FOR THIRD OBJECT #2------------------------------------------------------------------------
    ur5_pose_10 = geometry_msgs.msg.Pose()
    ur5_pose_10.position.x = 0.56917455176
    ur5_pose_10.position.y = -0.249971843201
    ur5_pose_10.position.z = 0.884687672761
    ur5_pose_10.orientation.x = -0.0473633344341
    ur5_pose_10.orientation.y = -0.704158279602
    ur5_pose_10.orientation.z = 0.706216263776
    ur5_pose_10.orientation.w = 0.0563597427025 
    lst_joint_angles_10 = [math.radians(-33.8227879145),
                          math.radians(-57.7970579219),
                          math.radians(99.9376617039),
                          math.radians(-131.58926836),
                          math.radians(-90.5052154393),
                          math.radians(64.5918976813)]

    t_pose = threading.Thread(target=ur5.go_to_pose, args=(ur5_pose_10,)) 
    t_joint_angle = threading.Thread(target=ur5.set_joint_angles, args=(lst_joint_angles_10,))
    
    t_pose.start() 
    t_joint_angle.start()
    t_pose.join() 
    t_joint_angle.join()
    ur5.go_to_predefined_pose("gripperP2")


     # FOR THIRD OBJECT #3------------------------------------------------------------------------
    ur5_pose_11 = geometry_msgs.msg.Pose()
    ur5_pose_11.position.x = 0.572932964795
    ur5_pose_11.position.y = -0.249413244802
    ur5_pose_11.position.z = 1.18089891698
    ur5_pose_11.orientation.x = -0.0473702783025
    ur5_pose_11.orientation.y = -0.704158047232
    ur5_pose_11.orientation.z =  0.706216458893
    ur5_pose_11.orientation.w = 0.0563543648801
    lst_joint_angles_11 = [math.radians(-33.5846646784),
                          math.radians(-72.3922440137),
                          math.radians(74.2090627053),
                          math.radians(-91.2642748998),
                          math.radians(-90.5024595455),
                          math.radians(64.8301397328)]

    t_pose = threading.Thread(target=ur5.go_to_pose, args=(ur5_pose_11,)) 
    t_joint_angle = threading.Thread(target=ur5.set_joint_angles, args=(lst_joint_angles_11,))
    
    t_pose.start() 
    t_joint_angle.start()
    t_pose.join() 
    t_joint_angle.join()

     # FOR THIRD OBJECT #4------------------------------------------------------------------------
    ur5_pose_12 = geometry_msgs.msg.Pose()
    ur5_pose_12.position.x = 0.131386457404
    ur5_pose_12.position.y = -0.693603615502
    ur5_pose_12.position.z = 1.12369347783
    ur5_pose_12.orientation.x = -0.047360459177
    ur5_pose_12.orientation.y = -0.70419796818
    ur5_pose_12.orientation.z = 0.706181035071
    ur5_pose_12.orientation.w = 0.0563076746372
    lst_joint_angles_12 = [math.radians(-88.1683698512),
                          math.radians(-60.0746957954),
                          math.radians(65.0386638833),
                          math.radians(-95.0475394927),
                          math.radians(-90.7378606535),
                          math.radians(10.2388802531)]

    t_pose = threading.Thread(target=ur5.go_to_pose, args=(ur5_pose_12,)) 
    t_joint_angle = threading.Thread(target=ur5.set_joint_angles, args=(lst_joint_angles_12,))
    
    t_pose.start() 
    t_joint_angle.start()
    t_pose.join() 
    t_joint_angle.join()
    ur5.go_to_predefined_pose("gripperZeros")
    
    del ur5


if __name__ == '__main__':
    main()
