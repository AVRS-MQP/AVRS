AVRS-MQP: abb_motion_actionserver
Maintainer: avrs.mqp@gmail.com
Authors: Nikolas Gamarra, Ryan O'Brien

This node connects with an Arduino Yun which is simulating an electric vehicle.
The Yun sends a packet of information over Wi-Fi describing either a Tesla, a
Volt, or a Leaf. The info is written to a text file, read by the node, and made
into a message to be published by coms_node. Coms_uplink is a test node that
handles the coms messages that was later made a part of main.

```
Required Libraries: coms_msgs
```
