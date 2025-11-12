# Overview

## Problem:
Linux assigns /dev/video* numbers dynamically each boot, so identical USB cameras may swap device indices (e.g. what was /dev/video0 yesterday becomes /dev/video2 today).
ROS 2 and OpenCV nodes that hard-code /dev/videoN then connect to the wrong camera or fail entirely.
## Solution:
Use udev rules to create persistent, human-readable symlinks like /dev/cam_left and /dev/cam_right bound to each camera’s unique USB serial number.
These stable names stay constant across reboots, ensuring the correct camera is always launched regardless of enumeration order.

Create stable names like /dev/cam_left, /dev/cam_right.

    v4l2-ctl --list-devices
    udevadm info --name=/dev/video0 | grep ID_SERIAL

Each camera has a unique serial (e.g. 046d_Logitech_Webcam_C930e_ABC12345).

Add a rule /etc/udev/rules.d/99-cameras.rules :

    SUBSYSTEM=="video4linux", ATTRS{serial}=="ABC12345", SYMLINK+="cam_left"
    SUBSYSTEM=="video4linux", ATTRS{serial}=="DEF67890", SYMLINK+="cam_right"

Reload:

    sudo udevadm control --reload-rules && sudo udevadm trigger

