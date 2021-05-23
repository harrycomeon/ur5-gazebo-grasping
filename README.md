

##### 1,problem：

1-1, 如果存在robotiq_85_gripper  包编译存在问题 用这个包替换

git clone https://github.com/StanleyInnovation/robotiq_85_gripper.git


#### 2，Description：

在GAZEBO环境下，搭建了世界环境，并且添加了深度相机和带手抓的UR5机械臂，结合视觉算法，进行物体位姿估计，利用moveit_commander实现简单抓取操作。

参考网址：

https://blog.csdn.net/harrycomeon/article/details/107073020
https://blog.csdn.net/harrycomeon/article/details/107331674


#### 3,Test

### 首先测试模拟环境下，脱离视觉的情况下，利用ROS的moveit控制机械臂进行轨迹规划，实现抓取的操作过程。

运行语句：

启动模拟环境：
roslaunch ur5_single_arm_tufts ur5_single_arm_gazebo.launch
结合抓取策略，进行物块的抓取操作
rosrun ur5_single_arm_manipulation grasping_demo.py

###  视觉抓取演示

利用视觉获取物块的位姿，利用ROS的moveit控制机械臂进行轨迹规划，实现静态抓取的操作过程。
运行语句：

启动模拟环境：
roslaunch ur5_single_arm_tufts ur5_single_arm_gazebo.launch

识别物块位姿，并发布到TF树
roslaunch find_object_2d find_object_3d_kinect2.launch

订阅发布的位姿，并发布给机械臂
rosrun opencv tf_listener.py

结合抓取策略，进行物块的抓取操作
rosrun ur5_single_arm_manipulation grasping_demo_vision2.py 


#### 4,Tips：

1，对比观察一下，视觉算法获取的位姿与实际布置的位姿的位姿偏差。

2，订阅发布的TF树中，坐标变换时，注意根据发布的话题名称修改程序。

3，思考一下，手眼标定在整个静态抓取中所扮演的角色。

4，从程序中理解，每个部分的实现过程。

感谢ziyu同学和其他github上的参考代码及文档，才有了现在的静态抓取，开源棒棒棒。

参考网址：https://github.com/Dzy-HW-XD/kinectv2_ur5












 
