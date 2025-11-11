# Shortlist for your stack. Everything else is noise.

## Robot description + viz

ros-humble-xacro – build URDFs with macros.
ros-humble-urdf, ros-humble-urdfdom, ros-humble-urdfdom-headers, ros-humble-urdfdom-py, ros-humble-kdl-parser – URDF parsing.
ros-humble-joint-state-publisher, ros-humble-joint-state-publisher-gui – fake joints for RViz and debugging.
ros-humble-robot-state-publisher – publish TF from URDF.
ros-humble-vision-msgs, ros-humble-visualization-msgs, ros-humble-geometry-msgs, ros-humble-sensor-msgs – message types you’ll touch constantly.
ros-humble-interactive-markers, ros-humble-rqt-graph, ros-humble-rqt-tf-tree, ros-humble-rqt-image-view, ros-humble-rqt-plot, ros-humble-rqt-console – quick inspection tools.

## TF, math, point clouds

ros-humble-tf2, ros-humble-tf2-ros, ros-humble-tf2-eigen, ros-humble-tf2-geometry-msgs, ros-humble-tf2-py, ros-humble-tf-transformations – frame math and conversions.
ros-humble-pcl-ros, ros-humble-pcl-conversions, ros-humble-pcl-msgs, ros-humble-point-cloud-transport, ros-humble-point-cloud-transport-plugins, ros-humble-open3d-conversions – cloud IO and conversions.
ros-humble-octomap, ros-humble-octomap-ros, ros-humble-octomap-msgs, ros-humble-octomap-rviz-plugins – only if you’ll build 3D occupancy.

## Cameras and images

ros-humble-image-pipeline (pulls image_proc, image_transport, plugins), ros-humble-image-geometry, ros-humble-image-tools, ros-humble-image-view.
ros-humble-camera-calibration, ros-humble-camera-calibration-parsers, ros-humble-camera-info-manager.

## Realsense: ros-humble-librealsense2, ros-humble-realsense2-camera, ros-humble-realsense2-camera-msgs, ros-humble-realsense2-description.

## AprilTag + perception utilities

ros-humble-apriltag, ros-humble-apriltag-ros, ros-humble-apriltag-msgs, ros-humble-apriltag-tools, ros-humble-apriltag-draw.

Optional extras you may like: ros-humble-ffmpeg-image-transport, ros-humble-foxglove-bridge, ros-humble-foxglove-msgs.

## Manipulation

ros-humble-moveit meta pulls: moveit-core, moveit-ros-*, moveit-plugins, moveit-visual-tools, moveit-servo, moveit-setup-assistant.
ros-humble-geometric-shapes, ros-humble-srdfdom, ros-humble-ompl come along; keep them.
ros2_control for xArm or any arm
ros-humble-ros2-control, ros-humble-ros2-controllers, ros-humble-ros2-control-test-assets.

## Controllers you’ll actually use:
ros-humble-joint-state-broadcaster, ros-humble-joint-trajectory-controller, ros-humble-position-controllers, ros-humble-effort-controllers, ros-humble-forward-command-controller, ros-humble-range-sensor-broadcaster, ros-humble-force-torque-sensor-broadcaster.

If sim with Gazebo(Fortress/Garden): ros-humble-gz-ros2-control or ros-humble-gazebo-ros2-control depending on sim stack.

## Simulation (only if you sim)

Gazebo Garden/Fortress line: ros-humble-ros-gz, ros-humble-ros-gz-sim, ros-humble-ros-gz-bridge, ros-humble-ros-gz-image, ros-humble-ros-gz-interfaces.
Do not mix with ros-ign-* set in the same env.

Classic Gazebo line: ros-humble-gazebo-ros-pkgs, ros-humble-gazebo-plugins, ros-humble-gazebo-ros.

## Bags, tracing, debugging

ros-humble-rosbag2 + storage: ros-humble-rosbag2-storage-mcap, ros-humble-rosbag2-compression-zstd.
ros-humble-rosbridge-suite if you want WS JSON.
ros-humble-tracetools, ros-humble-tracetools-analysis if you profile.

## RMW and middleware

ros-humble-rmw-fastrtps-cpp and/or ros-humble-rmw-cyclonedds-cpp. Pick one default per env to avoid weird QoS issues. Zenoh is experimental: ros-humble-rmw-zenoh-cpp.

## Optional math/kinematics

ros-humble-pinocchio, ros-humble-gtsam, ros-humble-hpp-fcl, ros-humble-geodesy only if you need them.
