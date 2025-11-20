# Mecanumbot core package 

**Function:** process OpenCR's in and outputs

---



## Nodes:

#### mecanumbot_IO_node:

#####  **Function:**

 receieves the state elements of the OpenCR board, converts them into a custom message. Takes motor command messages and sends them to the board to process.

##### **Subscribers:**

* ***cmd_vel***: custom velocity commands for X (ahead-back), Y (left-right) linear and Z angular velocities
* ***cmd_accessory_pos:*** custom position commands for the accessory motors (neck and 2 grabber states)

##### **Publishers:**

* ***opencr_state:*** the data found inside a sigle packet of the board's messages, divided to individual state (see mecanumbot_msgs)

---

#### mecanumbot_sensorproc_node

##### **Function:**

Converts raw OpenCR data packet message to sensor states 

##### **Subscribers:**

* ***opencr_state:*** the data found inside a sigle packet of the board's messages, divided to individual state (see mecanumbot_msg)

##### Publishers:

* ***odom:*** a nav_msgs/Odometry type, information about robot odometry (positon and velocity)
* imu: a sensor_msgs/Imu type, velocity and acceleration information gained straight from the robot's IMU sensor.
* ***battery_state:*** a sensor_msgs/BatteryState type, calculates battery state from the board's detected voltage.
* ***joint_state:* **a sensor_msgs/JointState type, describes wheel positions and velocities. The effort part of the message is left blank.
