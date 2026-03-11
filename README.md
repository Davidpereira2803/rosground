# ROSmonitor

A React Native mobile application for monitoring and controlling ROS (Robot Operating System) robots via rosbridge WebSocket connection and live MJPEG video streaming.

## Overview

ROSmonitor is an Expo-managed React Native application that allows you to:
- Connect to a ROS system via rosbridge WebSocket protocol
- Subscribe to ROS topics and monitor live telemetry data
- Display real-time sensor data and robot state information
- View live MJPEG video streams from robot cameras (coming in future release)

## Features

- **WebSocket Connection**: Connects to rosbridge server for real-time ROS communication
- **Topic Subscription**: Manually subscribe to any ROS topic by specifying topic name and type
- **Live Telemetry**: View incoming ROS messages as formatted JSON in real-time
- **Setup Guide**: Built-in how-to guide with visual hierarchy and icons for easy onboarding
- **Dark Theme**: Modern dark blue interface optimized for outdoor use and battery life
- **Cross-Platform**: Runs on both iOS and Android devices

## Prerequisites

### On Your Robot/Computer

1. **ROS Installation** (ROS 2 or ROS 1)
   - Ensure ROS is installed on your system: https://docs.ros.org/

2. **rosbridge_server Package**
   - Install rosbridge for WebSocket communication:
   ```bash
   # For ROS 2 (Humble or newer)
   sudo apt install ros-${ROS_DISTRO}-rosbridge-server
   
   # For ROS 1
   sudo apt install ros-${ROS_DISTRO}-rosbridge-server
   ```

3. **Source ROS Environment**
   - Before launching rosbridge, source your ROS setup:
   ```bash
   source /opt/ros/<your-ROS-distro>/setup.bash
   ```

4. **Start rosbridge_server** (runs on port 9090 by default)
   ```bash
   # For ROS 2
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   
   # For ROS 1
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```
   - Verify it's running: You should see output indicating the WebSocket bridge is listening

5. **Network Setup**
   - Ensure your robot/computer IP is accessible from your mobile device
   - Both devices must be on the same WiFi network
   - Ports 9090 (rosbridge) and 8080 (optional video streaming) should be accessible
   - Check firewall settings if you have connection issues

### On Your Mobile Device

1. **Node.js** (v16 or later) — [Download here](https://nodejs.org/)

2. **Expo CLI**
   ```bash
   npm install -g expo-cli
   ```

3. **Expo Go App** (for development/testing)
   - iOS: Available on the App Store
   - Android: Available on Google Play
   - Alternative: Build a development version for local network access on iOS

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

For iOS testing against a robot on your LAN, prefer a development build or production build over Expo Go. The local-network and App Transport Security settings in [app.json](./app.json) are only applied to your own iOS app bundle.

## Usage

### Connecting to Your Robot

1. Launch the app
2. On the Connect screen, enter:
   - **Robot IP**: The IP address of your robot or computer running ROS (e.g., 192.168.1.100)
   - **Rosbridge Port**: Port where rosbridge_server is running (default: 9090)
   - **Video Port**: Reserved for future video streaming feature (default: 8080)
3. Tap **CONNECT**
4. Need help? Tap "View Setup Guide" link at the bottom of the screen

### Subscribing to Topics

1. From the Dashboard, tap **ADD TOPIC**
2. Enter the topic details:
   - **Topic Name**: Full topic path (e.g., /battery, /odom, /cmd_vel)
   - **Topic Type**: Message type (e.g., std_msgs/msg/Float32, nav_msgs/msg/Odometry)
3. Tap **SUBSCRIBE**
4. The topic will appear on the Dashboard with live data updates

### Viewing Data

- **Topic Widgets**: Show the most recent message for each subscribed topic
- **JSON Format**: Messages are displayed as formatted JSON for inspection
- **Real-time Updates**: Data refreshes automatically as new messages arrive

### Unsubscribing from Topics

- Tap the **X** button on any topic widget to unsubscribe

## Project Structure

```
rosground/
├── App.js                          # Main app entry point with navigation
├── docs/                           # Documentation in HTML/CSS format
│   ├── assets/
│   ├── index.html
│   └── styles.css
├── src/
│   ├── context/
│   │   └── ROSContext.js          # WebSocket connection and state management
│   ├── screens/
│   │   ├── ConnectScreen.js       # Connection configuration screen
│   │   ├── DashboardScreen.js     # Main dashboard with topics
│   │   ├── TopicBrowserScreen.js  # Topic subscription interface
│   │   ├── PublishScreen.js       # Topic publishing interface
│   │   └── HowToUseScreen.js      # Setup guide and troubleshooting
│   ├── components/
│   │   └── VideoPanel.js          # MJPEG video display component (future)
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

### Video Streaming (Future Update/Feature)

- Uses React Native Image component with MJPEG stream URL
- Falls back to "Not connected" state when stream unavailable
- Displays stream URL for debugging purposes

## Network Configuration

Ensure your mobile device and robot are on the same network:

1. **WiFi**: Connect both devices to the same WiFi network
2. **Firewall**: Ensure ports 9090 and 8080 are accessible
3. **IP Address**: Use the robot's local IP address (not localhost)
4. **iOS Permission Prompt**: Accept the local network permission prompt the first time the app connects to the robot on iPhone or iPad

## Troubleshooting

### Cannot Connect to Robot

- Verify the robot's IP address is correct
- Check that rosbridge_server is running: `ros2 topic list` should show topics
- Ensure firewall allows connections on port 9090
- Confirm both devices are on the same network

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
- **MJPEG Video Streaming**: Live camera feed from robot
- Auto-discovery of available topics
- Custom message formatters for common message types
- Joystick/gamepad control interface
- Save/load topic configurations
- Multiple robot connections
- Recording and playback of telemetry data

## Version

**Current**: Beta v1.0.3

**v1.0.3 Highlights**:
- **Gesture-Based Navigation**: Implemented swipe-to-go-back gesture navigation throughout the app for modern mobile UX
- **Unified Headers**: Standardized header structure across HowToUseScreen, TopicBrowserScreen, and PublishScreen with consistent typography (18px title, 12px subtitle)
- **Robot/PC Setup Guide**: Added detailed robot/PC setup commands to HowToUseScreen including ROS installation, environment sourcing, and rosbridge launch instructions
- **Website Link**: Added website link in ConnectScreen footer for easy access to app documentation and resources
- **Improved Header Layout**: Better visual separation between header sections with refined spacing and alignment
- **Removed Redundant Footers**: Eliminated footer close buttons in favor of gesture-based navigation as primary interaction method

**v1.0.2 Highlights**:
- Enhanced "How to Use" screen with visual hierarchy and icons
- Improved section organization with material design icons
- Refined color theme with better contrast and visual balance
- Footer close button for consistent navigation across screens
- Better UX for onboarding and troubleshooting guidance

**v1.0.1 Highlights**:
- Added built-in "How to Use" setup guide
- Improved connection screen with help link
- Updated documentation to clarify video streaming as future feature
- Cleaner UI flow for first-time users

**v1.0.0 Highlights**:
- WebSocket rosbridge connection
- Manual topic subscribe and unsubscribe
- Live telemetry JSON viewer
- Dark theme UI