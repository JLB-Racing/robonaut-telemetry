# robonaut-telemetry

## Building from source

### Fetch source and install dependencies

```bash
cd <path/to/your/ros_ws>
git clone https://github.com/foxglove/ros-foxglove-bridge.git src/ros-foxglove-bridge
rosdep update
rosdep install --ignore-src --default-yes --from-path src
```

### ROS 2
```
colcon build --event-handlers console_direct+ --symlink-install
source install/local_setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
