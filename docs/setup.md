---
layout: page
title: Setup
---

This guide helps you connect ROS Monitor to your ROS machine in a few steps.

## 1. Install rosbridge

ROS 2:

    sudo apt install ros-${ROS_DISTRO}-rosbridge-suite

ROS 1:

    sudo apt install ros-${ROS_DISTRO}-rosbridge-server

## 2. Source your ROS environment

ROS 2 example:

    source /opt/ros/${ROS_DISTRO}/setup.bash

If you use a workspace:

    source ~/your_ws/install/setup.bash

## 3. Start rosbridge websocket

ROS 2:

    ros2 launch rosbridge_server rosbridge_websocket_launch.xml

ROS 1:

    roslaunch rosbridge_server rosbridge_websocket.launch

Default port is 9090.

## 4. Optional camera stream

If you want camera preview in the app, run the helper server from this repository:

    python3 video__http_server.py --topic /camera/image_raw --port 8080

Adjust topic and port as needed.

## 5. Network checklist

- Phone and ROS machine must be on the same local network
- Open firewall access for port 9090
- If using video preview, also open port 8080
- Use machine local IP, not localhost

## 6. Connect from app

- Host: your machine IP
- Rosbridge port: 9090
- Video port: 8080 (optional)

After connecting, go to topic browsing and subscribe to the topics you want to monitor.