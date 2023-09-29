# AVG-Localization
## 概述
  这是一个基于多传感器融合的机器人定位算法，涉及的传感器有惯性测量单元（IMU）和激光雷达。算法运行平台为matlab2021b与CoppeliaSim Edu 4.4.0。算法的结构如下图。
![image](https://github.com/untitled-man/AVG-Localization/blob/main/pictures/%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F%E7%BB%93%E6%9E%84.drawio.png)
  位姿预测部分用REKF算法将激光雷达与IMU融合，位姿更新部分用KLD-MCL将激光雷达与预测信息融合，最总得到机器人的运动轨迹。REKF是在EKF的基础上改进得到的，KLD-MCL则是基于MCL改进的，MATLAB文件夹中包含了改进前后的算法，可供比较各种算法的效果差异。

## 使用说明
  将MATLAB文件夹中的文件都放在一个文件夹下，运行main.m文件即可。
