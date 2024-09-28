# grabber_msg_interface
grabber_msg_interface is an interface package containing the message definition for controlling grabbers.

**Note:** it is recommended to build the whole workspace at the same time, following the instructions [here](/README.md).

## Usage

### 1. Build package if necessary:
```
colcon build
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

### 4. Message interface ready
Now you can use this interface to implement GrabberPosition msg, or build [mecanum_teleop](/mecanum_teleop/README.md) package, where it is already implemented.