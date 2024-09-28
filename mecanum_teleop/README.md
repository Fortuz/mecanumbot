# mecanum_teleop
mecanum_teleop is a teleoperation node that publishes linear x and y, and angular z velocities to cmd_vel topic in ROS2. It was custom made for mecanum wheeled robots.

**Note:** it is recommended to build the whole workspace at the same time, following the instructions [here](/README.md).

## Usage

### 0. Prequisites:
Package [grabber_msg_interface](../grabber_msg_interface/README.md) must be built to use this package.

### 1. Enter folder:
```
cd mecanum_teleop
```

### 2. Build package if necessary: (both Windows and Linux)
```
colcon build --symlink-install --merge-install
```

### 3. Source package:

**Windows:**
```
call install/setup.bat
```
**Linux:**
```
source install/setup.bash
```

### 4. Run node:
```
ros2 run mecanum_teleop mecanum_teleop_key
```

### 5. Control your bot:
```
Moving around:
    q   w   e      u   i   o   p
    a   s   d
        x
    
[Incremental mode]:
    w/x : increase/decrease X linear velocity (max 0.26)
    a/d : increase/decrease Y linear velocity (max 0.26)
    q/e : increase/decrease angular velocity (max 1.82)

[Instant mode]:
    w/x : set X linear velocity to +/-0.18
    a/d : set Y linear velocity to +/-0.18
    q/e : set angular velocity to +/-1.20

u/i : open/close left grabber
p/o : open/close right grabber
space key, s : force stop
m : switch between modes

CTRL-C to quit
```