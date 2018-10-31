# skelevisors
Skeleton tracking using ROS

# Девиз
Человек как инструмент

# Цель
Управление роботом (или не роботом) движениями (или не движениями) тела (или телами) человека (или не человека).

# Используемые технологии
- ROS Kinetic
- Kinect (это не ROS Kinetic, это Kinect)
- OpenNI_tracker
- NITE
- Darwin robot
- https://github.com/avin2/SensorKinect 

## Используемые ROS-пакеты
- ros-gazebo
- darwin-gazebo

# Startup schedule
```
roslaunch openni_launch openni.launch device_id:=#2 depth_registration:=true
rosrun openni_tracker openni_tracker
roslaunch darwin_gazebo darwin_gazebo.launch
rosrun skelevisors subscriber.py 
```

# Описание
Кажую неделю мы приходим на кафедру и пытаемся что-то запустить, используя устаревшие туториалы и копируя чужой код. В будущем, мы надеемся управлять роботом Darwin с помощью тела человека (используя Kinect). Надеемся успеть до конца семестра, но не гарантируем.
