# mecanum_teleop
mecanum_teleop is a teleoperation node that publishes linear x and y, and angular z velocities to cmd_vel topic in ROS2. It was custom made for mecanum wheeled robots.

## Usage

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
    q   w   e
    a   s   d
        x

w/x : increase/decrease X linear velocity (~ 0.26)
a/d : increase/decrease Y linear velocity (~ 0.26)
q/e : increase/decrease angular velocity (~ 1.82)

space key, s : force stop

CTRL-C to quit
```