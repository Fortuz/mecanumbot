# mecanum_teleop

## Usage

1. Enter folder:
```
cd mecanum_teleop
```
2. Source package:

    - Windows: `call install/setup.bat`

    - Linux: `. install/setup.bash`

3. Run node:
```
ros2 run mecanum_teleop mecanum_teleop_key
```

 4. Control your bot:
    - w/x : increase/decrease X linear velocity (~ 0.26)
    - a/d : increase/decrease Y linear velocity (~ 0.26)
    - q/e : increase/decrease angular velocity (~ 1.82)