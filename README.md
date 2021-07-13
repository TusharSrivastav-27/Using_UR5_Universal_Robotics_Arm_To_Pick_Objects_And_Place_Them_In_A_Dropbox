# Using_UR5_Universal_Robotics_Arm_To_Pick_Objects_And_Placing_Them_In_A_Dropbox

The challenge posed in this task is Picking and Placing objects in their designated drop-boxes. We used MoveIt motion planning framework to place the objects in their corresponding drop box. Initially, the UR5(Universal Robot) arm will spawn at the origin having coordinates [0, 0,0.1] in Gazebo simulator. On the tool tip of UR5 a 2 finger gripper(2f Robotiq gripper) is connected.

UR5 arm should pick the following objects and drop them in the drop-boxes mentioned below:-

    Objects 1 should be dropped in to drop-box 1.

    Objects 2 and 3 should be dropped in to drop-box 2
    
    UR5 arm should plan a path in such a way that it never collides with tables and drop-boxes.
