# ROSground

A React Native mobile application for monitoring and controlling ROS (Robot Operating System) robots via rosbridge WebSocket connection and live MJPEG video streaming.

## Overview

ROSground is an Expo-managed React Native application that allows you to:
- Connect to a ROS system via rosbridge WebSocket protocol
- Subscribe to ROS topics and monitor live telemetry data
- View live MJPEG video streams from robot cameras
- Display real-time sensor data and robot state information

## Features

- **WebSocket Connection**: Connects to rosbridge server for real-time ROS communication
- **Topic Subscription**: Manually subscribe to any ROS topic by specifying topic name and type
- **Live Telemetry**: View incoming ROS messages as formatted JSON in real-time
- **Video Streaming**: Display MJPEG video streams from robot cameras
- **Dark Theme**: Modern dark blue interface optimized for outdoor use and battery life
- **Cross-Platform**: Runs on both iOS and Android devices

## Prerequisites

### On Your Robot/Computer

1. **ROS Installation**: ROS 2 or ROS 1 with rosbridge installed
2. **rosbridge_server**: Running on port 9090 (default)
   ```bash
   # For ROS 2
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   
   # For ROS 1
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```

3. **MJPEG Server** (optional for video): HTTP server streaming MJPEG on port 8080
   ```bash
   # Example using web_video_server (ROS 1)
   rosrun web_video_server web_video_server
   
   # Example using image_transport (ROS 2)
   ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_raw
   ```

### On Your Mobile Device

1. Node.js (v16 or later)
2. Expo CLI
3. Expo Go app (for testing) or build for production

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/Davidpereira2803/rosground
   cd rosground
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npx expo start
   ```

4. Scan the QR code with:
   - **iOS**: Camera app (opens Expo Go)
   - **Android**: Expo Go app

## Usage

### Connecting to Your Robot

1. Launch the app
2. On the Connect screen, enter:
   - **Robot IP**: The IP address of your robot or computer running ROS (e.g., 192.168.1.100)
   - **Rosbridge Port**: Port where rosbridge_server is running (default: 9090)
   - **Video Port**: Port for MJPEG video stream (default: 8080)
3. Tap **CONNECT**

### Subscribing to Topics

1. From the Dashboard, tap **ADD TOPIC**
2. Enter the topic details:
   - **Topic Name**: Full topic path (e.g., /battery, /odom, /cmd_vel)
   - **Topic Type**: Message type (e.g., std_msgs/msg/Float32, nav_msgs/msg/Odometry)
3. Tap **SUBSCRIBE**
4. The topic will appear on the Dashboard with live data updates

### Viewing Data

- **Video Feed**: Displays at the top of the Dashboard when connected
- **Topic Widgets**: Show the most recent message for each subscribed topic
- **JSON Format**: Messages are displayed as formatted JSON for inspection
- **Real-time Updates**: Data refreshes automatically as new messages arrive

### Unsubscribing from Topics

- Tap the **X** button on any topic widget to unsubscribe

## Project Structure

```
rosground/
├── App.js                          # Main app entry point with navigation
├── src/
│   ├── context/
│   │   └── ROSContext.js          # WebSocket connection and state management
│   ├── screens/
│   │   ├── ConnectScreen.js       # Connection configuration screen
│   │   ├── DashboardScreen.js     # Main dashboard with topics and video
│   │   └── TopicBrowserScreen.js  # Topic subscription interface
│   ├── components/
│   │   └── VideoPanel.js          # MJPEG video display component
│   └── theme/
│       └── colors.js              # Application color theme
├── package.json
└── babel.config.js
```

## Technical Details

### Rosbridge Protocol

The app uses the rosbridge WebSocket protocol to communicate with ROS:

- **Subscribe**: `{ "op": "subscribe", "topic": "/topic_name", "type": "msg_type" }`
- **Unsubscribe**: `{ "op": "unsubscribe", "topic": "/topic_name" }`
- **Message Format**: `{ "op": "publish", "topic": "/topic_name", "msg": {...} }`

### State Management

- React Context API manages WebSocket connection and subscribed topics
- Centralized message handling updates topic data in real-time
- Connection state persists across navigation

### Video Streaming

- Uses React Native Image component with MJPEG stream URL
- Falls back to "Not connected" state when stream unavailable
- Displays stream URL for debugging purposes

## Network Configuration

Ensure your mobile device and robot are on the same network:

1. **WiFi**: Connect both devices to the same WiFi network
2. **Firewall**: Ensure ports 9090 and 8080 are accessible
3. **IP Address**: Use the robot's local IP address (not localhost)

## Troubleshooting

### Cannot Connect to Robot

- Verify the robot's IP address is correct
- Check that rosbridge_server is running: `ros2 topic list` should show topics
- Ensure firewall allows connections on port 9090
- Confirm both devices are on the same network

### No Video Stream

- Verify MJPEG server is running on the specified port
- Check video URL in browser: `http://<robot-ip>:8080/stream`
- Ensure camera topic is publishing
- Note: Video streaming on React Native may be device-dependent

### Topics Not Updating

- Verify topic name and type are correct
- Check that the topic is actively publishing: `ros2 topic echo /topic_name`
- Ensure message type matches exactly (including namespace)

## Technology Stack

- **React Native**: Cross-platform mobile framework
- **Expo**: Development and build toolchain
- **React Navigation**: Screen navigation
- **WebSocket**: Real-time bidirectional communication
- **rosbridge**: ROS WebSocket protocol bridge

## License

This project is licensed under the MIT License.

## Contributing

Contributions are welcome. Please:
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## Support

For issues and questions:
- Check the Troubleshooting section above
- Review rosbridge documentation: http://wiki.ros.org/rosbridge_suite
- Open an issue in the repository

## Roadmap

Future enhancements:
- Auto-discovery of available topics
- Custom message formatters for common message types
- Joystick/gamepad control interface
- Publish to topics from the app
- Save/load topic configurations
- Multiple robot connections
- Recording and playback of telemetry data