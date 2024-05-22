# 最终效果测试
## PX4、MAVROS下载配置
[PX4的安装与基本环境的配置](https://zhuanlan.zhihu.com/p/394463440)

## 运行步骤
```
rango@rango-ThinkPad-X1-Extreme-Gen-4i:~/PX4-Autopilot$ make px4_sitl gazebo

rango@rango-ThinkPad-X1-Extreme-Gen-4i:~/PX4-Autopilot$ roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

rango@rango-ThinkPad-X1-Extreme-Gen-4i:~/catkin_ws$ rosrun chatgpt_px4 chatgpt_communication.py 

rango@rango-ThinkPad-X1-Extreme-Gen-4i:~/catkin_ws$ rosrun chatgpt_px4 takeoff.py 

rango@rango-ThinkPad-X1-Extreme-Gen-4i:~$ rviz
```
