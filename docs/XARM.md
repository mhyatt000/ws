# Minimal steps to run xarm_ros2

Based on `src/xarm_ros2/ReadMe.md`, this is the shortest path to get something running.

## 1) Prepare and build once
```bash
# from /home/mhyatt/ws
pixi install
pixi shell
pixi run build
```

## 2) Source the built workspace (every new terminal)
```bash
cd /home/mhyatt/ws
source install/setup.bash
```

## 3) Run a minimal simulation (no robot hardware)
```bash
ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py
```

## 4) Run on a real xArm instead (optional)
```bash
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=<ROBOT_IP>
```

## 5) If using xarm_api directly (optional)
```bash
# driver
ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=<ROBOT_IP>

# basic init
ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"
ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"
ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"
```

## Notes from upstream README
- Use the branch matching your ROS distro (this workspace is ROS 2 Humble).
- Source the workspace before launching anything.
- Set `ROS_DOMAIN_ID` to avoid LAN interference (this repo defaults to `69` in `env.sh`).
