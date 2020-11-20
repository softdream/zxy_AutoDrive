# 自动驾驶项目代码简介
## 一、开发环境
### 1. 硬件平台
  工控机，Intel I7 处理器。<br>
### 2. 操作系统
  Ubuntu18.04 <br>
### 3. 开发环境
  ROS-Melodic，PCL库。<br>

## 二、ROS Package描述
### 1. lidar_process 包
  此 package 包含了对激光雷达三维点云的处理，主要用于车辆前后障碍物的识别。<br>
  主要流程为点云滤波（降采样，直通滤波，cropBox滤波），地面分割（随机采样一致性算法），欧氏聚类识别出障碍物的点云簇，得到簇相对于激光雷达坐标系的位姿和所含点云个数，超过一定阈值的判定为障碍物。<br>
 
### 2. telecontrol 包
  此 package 是用来接收遥控座椅发过来的控制命令，并通过一个 topic（/controlCmd）发布出来，数据供底层车辆控制程序使用，遥控座椅和车辆通信是基于TCP协议的自定义消息格式帧，增添头，无校验。此外，此程序将底层采集的IMU数据发送给遥控座椅。<br>
  
### 3. MMWR 包 
  此 package 是用来驱动车辆前装的毫米波雷达，雷达采用的是大陆的ARS-408，通过 can 协议读取雷达数据。
