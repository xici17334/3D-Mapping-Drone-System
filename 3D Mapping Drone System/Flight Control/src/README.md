Drone Flight Control System (ESP32)



This project implements a quadrotor flight control system on ESP32.

It supports IMU-based attitude control, ToF altitude sensing, optical flow position estimation, and Wi-Fi UDP remote control.

All modules are written in C++ (Arduino framework) with modular structure for easy debugging and testing.



Main modules:



main.cpp: System entry and 400 Hz control loop



flight\_control.cpp: Motor mixing and PID stabilization



rc.cpp: UDP remote control and ToF telemetry



sensor.cpp: IMU + ToF + voltage + Kalman fusion



PMW3901\_Sensor.cpp: Optical-flow velocity estimation



telemetry.cpp: Flight data transmission



tof.cpp: Dual VL53LX ranging driver



imu.cpp: BMI270 and Madgwick AHRS



pid.cpp: Generic PID core



Simulation \& real test:



Use simulated sensor data to verify control logic.



Connect real IMU, ToF, and motors for flight tests.



Send RC commands via UDP (port 4210) to control attitude and altitude.



Result:

The system achieves stable hovering, real-time telemetry, and reliable UDP communication for both indoor simulation and actual flight.

