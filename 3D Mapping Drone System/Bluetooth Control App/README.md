Project Title: Bluetooth Drone Control App

 Overview

This Android application is designed to control a custom-built drone via Bluetooth communication with an onboard ESP32 controller.
It allows users to send flight commands wirelessly and monitor real-time altitude feedback from the drone.

Features

Takeoff & Landing: Tap buttons to arm, take off, or land safely.

Directional Control: Move the drone forward, backward, left, and right with precise control.

Real-time Altitude Display: Continuously receive and display altitude data (from the drone’s ToF or barometric sensor).

Connection Status: The app shows the Bluetooth connection state (connected/disconnected).

How It Works

Power on the drone and ensure the ESP32 Bluetooth module is active (e.g., UAV_BT or custom name).

Open the app and connect to the drone’s Bluetooth device.

Once connected, use the control interface:

Takeoff → Drone starts motors and hovers.

Land → Drone descends and disarms.

↑ ↓ ← → → Directional movement control.

The altitude value is updated in real-time on the main screen, ensuring safe operation.

Requirements

Android 8.0 or later

Bluetooth enabled

Drone equipped with ESP32 + IMU + ToF sensor

 Future Improvements

Add PID tuning interface for flight control.

Support for voice commands or joystick input.

Integration with Wi-Fi telemetry for extended data visualization.