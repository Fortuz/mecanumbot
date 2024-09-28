# mecanumbot
ROS2 packages for controlling Mecanumbot.

## Usage

### 1. Build packages in workspace
```
colcon build --merge-install
```

### 2. Source packages
**Windows:**
```
call install/setup.bat
```
**Linux:**
```
source install/setup.bash
```

### 3. Use packages
For example `mecanum_teleop` package can be run with:
```
ros2 run mecanum_teleop mecanum_teleop_key
```
