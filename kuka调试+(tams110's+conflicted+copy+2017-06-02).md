## Goal:  use kuka LWR do experiments
## 外部资源：
1. FastResearchInterface (FRI) library:
  如何编译，使用FRIlibrary 参考： http://cs.stanford.edu/people/tkr/fri/html/

## ros confige PC of KUKA
```
1.create ros workspace (*only need at first time*)
pwd     显示当前目录
source /opt/ros/indigo/setup.bash   source下
catkin_init_workspace .   初始化ros workspace
catkin_make

2. 下次运行时：
source /opt/ros/indigo/setup.bash
cd catkin_ws
source devel/setup.bash
catkin_make
```

## network configure (multiple computers)
don't need to source.
```
**tams110 (master):**
ssh tams110
ping yuchen
export ROS_MASTER_URI=http://192.168.0.2:11311

**yuchen:**
ssh yuchen
ping tams110
export ROS_MASTER_URI=http://192.168.0.2:11311
```


## KuKa operation(real-robot)：
```
1. power of kuka
   turn on: power button of controller
   turn off: power button of controller

2. panel usage:
    a. joint control : A1~A6 初始角度： 0， 90， 0， 0， 0，0， 0
        Cartesian control: X,Y, Z, A, B, (configure--6--set num to 1)
    b. 模式选择： T-teaching mode , o-automatic mode
    c. 改变语言： configuration --> 9 --> language --> English

2. start: KUKA and ros.
    a. 切换到： automatic model
    b. 运行 FRIcontrolFNH 脚本, 按按钮 1， Start the script with the grey and green buttons; the scripts stops at a point and should be started again by releasing and pressing again the green button。
    c. select program -> select velocity -> select run mode.
    d. S，I，R 灯都变绿，成功.
    ###
    a. stop runing of program: turn auto model --> teaching mode
    b. navigation/program : view program name or view program detail.


3. FRI + Rviz:
    a. 启动：
    roslaunch tams_lwr bringup.launch
    b. 查看末端受力： add-WrenchStamped --topic: /lwr/estimatedExternalWrench
    c. 查看joint torque： add-- effort-- topic: /lwr/estimatedExternalJointTorques

    Force/torque view:
    cd /informatik2/tams/home/deng/catkin_ws/src/TAMS_robot/tams_cml/ros_fri/scripts
    ./plot_external_wren.sh


4. 控制Gripper
    a. 通过server 控制
    fist run:
    rosservice call /wsg_50/homing
    then:
    rosservice call /wsg_50/grasp 36 20
    rosservice call /wsg_50/release 40 20

    b. 通过teleop 控制
    roslaunch tams_lwr np_teleop.launch

4. 使用laptop
    Unable to connect via TCP, please check the port and address used
```

## Kuka 编程（真实机器人-FRI library）：
```
1. joint position controller
发送命令：topic: /lwr/jointPositionGoal  --> msg: ros_fri_msgs/RMLPositionInputParameters
查看joint position: topic: /lwr_measuredJointPositions  --> msg: std_msgs/Float32MultiArray
    a. Normal提供： 基于 /lwr/jointPositionGoal 节点与消息的控制
    rosrun ros_fri lwr_movej deg by 0 10 0 0 0 10 0

    b. 基于FRIlibrary提供的API 控制：
    i: 将FRIlibrary编译成动态链接库： fril_dynamic； 在fri_zdeng package 中已经编译成功
    ii： 使用FRIlibrary提供的API
    joint position control: rosrun ros_fri movej_yuchen
    joint impedence control: rosrun ros_fri JointImpe_yuchen
    cartesian impedence control: rosrun ros_fri CarImpe_yuchen
```

## Kuka operation (simulation)
1.参考 kuka_lwr package: https://github.com/CentroEPiaggio/kuka-lwr , 的single_lwr_example package

2. (Normal provide)
```
a.启动 moveit, rviz
roslaunch tams_lwr bringup.launch
roslaunch tams_lwr move_group.launch

b. 只启动 moveit
roslaunch tams_lwr_wsg50_moveit demo.launch
```

3. 基于 **JointTrajectoryController** joint position_control：
```
roslaunch lwr_gazebo_yuchen lwr_gazebo_position_yuchen.launch

cd '/home/yuchen/catkin_ws/src/TAMS_robot/tams_cml/lwr_gazebo_yuchen/scripts'
python lwr_init.py
```

4. 基于 **JointTrajectoryController** joint effector_control: (测试没有成功！ 不知道能否直接发送effort！)
```
roslaunch lwr_gazebo_yuchen lwr_gazebo_effort_yuchen.launch
cd '/home/yuchen/catkin_ws/src/TAMS_robot/tams_cml/lwr_gazebo_yuchen/scripts'
python lwr_effort_init.py
```
