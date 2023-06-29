# msis_pcl
A ros node (both roscpp and rospy) that converts stonefish msis image or a ping360 echo message to PointCloud2 msg.

## Subscribes To
```sensor_msgs/Image``` if using Stonefish <br>
or <br>
```Ping360_msgs/SonarEcho``` if using Blue Robotics Ping360 Sensor
<p>

## Publishes
```sensor_msgs/PointCloud2``` <br>

## Nodes
Both ```msis_pcl.py``` and ```msis_pcl```have the same application.

```map.py``` takes in the image and converts to a costmap. (WIP)

## Launch Files

```start_cpp.launch``` launches cpp node, ```start_python.launch```, python node. <br>
```start_all.launch```, launches both for comparative applications.

## Config
enable whether the source is simulation (stonefish) or BlueRobotics Ping360. Recommended to use along with [Ping360 ROS1 Package](https://github.com/GSO-soslab/bluerobotics_ping360). 

