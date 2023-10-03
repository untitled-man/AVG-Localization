# AVG-Localization
## 概述
  这是一个基于多传感器融合的机器人定位算法，涉及的传感器有惯性测量单元（IMU）和激光雷达。算法运行平台为matlab2021b与CoppeliaSim Edu 4.4.0。算法的结构如下图。
![image](https://github.com/untitled-man/AVG-Localization/blob/main/pictures/%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F%E7%BB%93%E6%9E%84.drawio.png)
  位姿预测部分用REKF算法将激光雷达与IMU融合，位姿更新部分用KLD-MCL将激光雷达与预测信息融合，最总得到机器人的运动轨迹。REKF是在EKF的基础上改进得到的，KLD-MCL则是基于MCL改进的，MATLAB文件夹中包含了改进前后的算法，可供比较各种算法的效果差异。

## 使用说明
step.1 获取传感器数据
  打开car_left_rot.ttt文件，运行仿真环境。再打开car_connect_test.m文件，运行。此时CoppeliaSim开始运行，并向MATLAB发送传感器数据。注意需要手动停止。运行结束后保存imu_data，laser_data，pose_idea这三个数据。
step.2 建立环境地图
  手动建立，提供了临时脚本mapping.mlx用于测试。尽力了，有什么更好的方法欢迎提出。
step.3 定位
  将imu_data，laser_data，pose_idea，map，p，dist，pose这几个数据保存在一个mat文件中，与其他MATLAB文件放在一个目录下。运行main.m前修改一下第10行的文件名即可。

关键变量说明：
imu_data：imu数据矩阵，包含加速度与角速度，每一行表示该采样时刻三轴加速度值与三轴角速度值，顺序分别为x加速度、y加速度、z加速度、x角速度、y角速度、z角速度
laser_data：激光雷达数据，行表示不同的采样时刻，列表示每一束激光的测量结果。激光雷达的参数可以在CoppeliaSim中修改
pose_idea：记录机器人的真实位姿，用于定位结果比较
map：栅格地图
p：栅格据图的矩阵表示，用于似然域计算
dist：预计算的似然域，记录每个栅格到最近的障碍物的距离，用于传感器模型的计算
pose：定位算法产生的位姿，一列为一个采样时刻，顺序为x，y，航向角

  目前算法性能较差，做不到实时定位，只能预先存下传感器数据。
