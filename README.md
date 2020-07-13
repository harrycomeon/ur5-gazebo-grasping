# ur5-gazebo-grasping

ur5+robotiq_85_gripper gazebo grasping

运行（run）：

roslaunch ur5_single_arm_tufts ur5_single_arm_gazebo.launch

rosrun ur5_single_arm_manipulation grasping_demo.py

描述：

在GAZEBO环境下，搭建了世界环境，并且添加了深度相机和带手抓的UR5机械臂，利用moveit_commander实现简单抓取操作。
