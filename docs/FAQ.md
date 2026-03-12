---
layout: page
title: FAQ
---

## Which ROS versions are supported?

ROS 1 and ROS 2 are supported through rosbridge.

## Which message types are supported?

Common message types render as JSON. Unknown or custom messages are shown as raw JSON.

## Why can’t the app connect?

Most connection issues come from one of these:

- Wrong host IP
- Wrong port
- Phone and ROS machine on different networks
- Firewall blocking port 9090

## Why is video not showing?

Check that:

- Video server is running
- Topic name is correct
- Video port matches app settings
- Port is reachable from phone

## Where do I report bugs or request features?

Open an issue on GitHub:
https://github.com/Davidpereira2803/rosground/issues