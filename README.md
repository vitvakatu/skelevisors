# skelevisors
Skeleton tracking using ROS

# Девиз
Человек как инструмент

# Цель
Управление роботом движениями тела человека.

# Используемые технологии
- ROS Melodic
- Kinect
- OpenNI_tracker
- [NITE](https://mega.nz/#!epNHBKzK!1-jcF_1tQtoYw9FSYmsKdPMqnPU2W-YdTJgA-f0ynnI)
- Darwin robot
- https://github.com/avin2/SensorKinect 

## Используемые ROS-пакеты
- ros-gazebo
- darwin-gazebo

# Startup schedule
```
roscore
roslaunch openni_launch openni.launch device_id:=#2 depth_registration:=true
rosrun openni_tracker openni_tracker
roslaunch darwin_gazebo darwin_gazebo.launch
rosrun skelevisors subscriber.py 
```

# Описание
Высокоточный манипулятор верхними конечностями и головой профессионального хирурга.
