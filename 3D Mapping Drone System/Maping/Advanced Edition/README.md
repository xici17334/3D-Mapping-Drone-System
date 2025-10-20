Drone MAPPING AND ODOMETRY (ROS2)



This project combines a 2D LiDAR and an altitude sensor to build a mapping and odometry system in ROS2.

It creates a full TF chain:

odom → base\_link → base\_link\_z → laser

where odom→base\_link comes from LiDAR odometry, and base\_link→base\_link\_z comes from altitude data.



Main scripts:



altitude\_to\_tf.py: converts altitude data to a TF transform.



icp\_odometry\_only.py: calculates odometry from LiDAR scans using ICP.



lidar\_udp\_bridge.py: receives LiDAR data over UDP and publishes /scan.



udp\_alt\_bridge.py: receives altitude over UDP (“ALT:value”) and publishes /altitude.



fake\_scan.py and fake\_altitude.py: generate simulated LiDAR and altitude data for testing.



scan\_to\_points3d1.py: projects 2D scans into 3D points for visualization.



HOW TO RUN WITH SIMULATED DATA



Open several terminals and source ROS2:

source /opt/ros/humble/setup.bash



Start these nodes:

python3 fake\_altitude.py

python3 fake\_scan.py

python3 altitude\_to\_tf.py

python3 icp\_odometry\_only.py

(optional) python3 scan\_to\_points3d1.py



Open RViz2.

Set the fixed frame to “odom” and add TF, LaserScan (/scan), and Odometry (/odom\_icp).



HOW TO RUN WITH REAL DATA



Upload the Arduino code to ESP32. It sends LiDAR frames to port 9000 and altitude as “ALT:value” to port 8888.



Connect your PC to the ESP32 Wi-Fi network.



Run the bridges and processing nodes:

python3 udp\_alt\_bridge.py

python3 lidar\_udp\_bridge.py

python3 altitude\_to\_tf.py

python3 icp\_odometry\_only.py

(optional) python3 scan\_to\_points3d1.py



Open RViz2 and visualize the map and movement.



This system can work fully in simulation or with real hardware.

It converts sensor data into ROS topics and TFs, allowing mapping, altitude variation, and odometry estimation in one integrated framework.

