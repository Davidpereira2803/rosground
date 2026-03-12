---
layout: page
title: FAQ
permalink: /faq/
---

## Which ROS versions are supported?

ROS Monitor supports ROS 1 and ROS 2 through rosbridge.

## What do I need before connecting?

You need:

- A running rosbridge server on your ROS machine
- Phone and ROS machine on the same local network
- Correct host IP and port in the app (default rosbridge port is 9090)

## Which message types are supported?

Common ROS messages are shown in readable JSON format.
Custom or unknown message structures are also displayed as JSON.

## Why can’t the app connect?

Most connection issues come from:

- Wrong host IP
- Wrong port
- Phone and ROS machine on different networks
- Firewall blocking port 9090
- rosbridge server not running

## Why are topics not updating?

Check that:

- You subscribed to the correct topic name
- The topic is actively publishing data
- The selected message type matches the topic type
- Connection is still active in the app

## Does ROS Monitor collect personal data?

No. ROS Monitor does not collect personal data.
Connection settings are stored locally on your device.

## Where do I report bugs or request features?

Open an issue on GitHub:
[https://github.com/Davidpereira2803/rosground/issues](https://github.com/Davidpereira2803/rosground/issues)