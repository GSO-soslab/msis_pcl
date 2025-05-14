# msis_pcl
A rclcpp node that converts stonefish msis image or a ping360 echo message to PointCloud2 msg.

## Subscribes To
```sensor_msgs/Image``` if using Stonefish <br>
or <br>
```Ping360_msgs/SonarEcho``` if using Blue Robotics Ping360 Sensor
<p>

## Publishes
```sensor_msgs/PointCloud2``` <br>

## Launch Files
```msis_pcl.launch.py``` launches the node with the param file ```params.yaml``` 

## Config
enable whether the source is simulation (stonefish) or BlueRobotics Ping360. Recommended to use along with [Ping360 ROS Package](https://github.com/GSO-soslab/bluerobotics_ping360). 

