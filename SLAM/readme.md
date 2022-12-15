We recorded the sensor data from the autonomous vehicle into a rosbag where the lidar data comes under the lidar1 topic. 

Open catkin_ws/src/hector_slam/hector_mapping/launch/mapping_default.launch file and change the default value of arg name="base_frame" and arg name="odom_frame", both to lidar1.

Finally when running, map the lidar1/scan topic to /scan topic.

Indoor SLAM
rosbag play insidehighbay.bag /lidar1/scan:=/scan --clock -r 2.5
Note: The clock argument just runs the clock through the bag faster.

Outdoor SLAM
rosbag play outsidehighbay.bag /lidar1/scan:=/scan --clock -r 2.5