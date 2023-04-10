# camera calibration
概述：ROS2中实现Realsense D435i的相机标定；

[参考资料](https://www.bilibili.com/video/BV19f4y1h73E?vd_source=6f2fc443cc3323efe0300c7cb662e8a8)，原项目基于ROS1实现；这里在ROS2中进行了实现；

仿真实现内容：
1. 【完成】在gazebo中导入相机；
2. 【完成】通过ros节点订阅相机发布的数据，并通过rviz展示数据；；
3. 【完成】创建Realsense D435i相机
4. 【完成】在仿真环境中订阅图像数据（【已修复】有一个小bug，订阅到数据的时间戳不一致；）
5. 【完成】完成相机的标定；
