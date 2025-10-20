Done LiDAR and Altitude Mapping System (ROS2)



This system combines a 2D LiDAR and an altitude sensor to create a lightweight 3D mapping setup in ROS2.

The coordinate chain is:

world → base\_link → laser

The altitude node updates the z-position of base\_link, while the LiDAR provides horizontal scans.



Main scripts:



Arduino.txt: ESP32 sends LiDAR frames (UDP 9000) and altitude data (UDP 8888, “ALT:value”).



udp\_alt\_bridge.py: receives altitude via UDP and publishes /altitude.



altitude\_to\_tf.py: converts /altitude to a TF transform (world→base\_link).



lidar\_udp\_bridge.py: converts UDP LiDAR frames into /scan messages.



scan\_to\_points3d1.py: combines /scan and TF to generate /points\_3d (3D cloud).



To test with simulated data:

source /opt/ros/humble/setup.bash

python3 udp\_alt\_bridge.py

python3 lidar\_udp\_bridge.py

python3 altitude\_to\_tf.py

python3 scan\_to\_points3d1.py

Then open RViz2, set “world” as Fixed Frame, and add TF, LaserScan, and PointCloud2.



With real data:

Upload Arduino code to ESP32, connect your PC to its Wi-Fi, and run the same nodes.



This system enables simple 3D reconstruction using only a LiDAR and altitude sensor.

