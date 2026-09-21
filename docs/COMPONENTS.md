# Mecanumbot component list

The parts that make up the current (Jetson Orin Nano) revision of the robot, with a
shop link for each one. Printed parts and parts built in-house are flagged **CUSTOM**.

**Prices** are single-unit list prices checked on **2026-09-18**. They exclude VAT and
shipping, and ROBOTIS prices differ by region. A `~` marks an estimate for a generic part
where the price depends on the seller. Re-check the prices before ordering.

**Evidence.** Each row comes from something in the repositories: the motor table and
firmware in [`mecanumbot_microcontrollers`](https://github.com/Fortuz/mecanumbot_microcontrollers),
the udev rules in `mecanumbot_description/udev/`, the wheel geometry in
`mecanumbot_core/mecanumbot_IO_node.py`, the STLs in `mecanumbot_description/meshes/3Dparts/`,
or the photos in `images/mecanumbot.jpg` and the microcontroller repo's
`Mecanumbot_MotorIDs.png`. Rows marked **TBC** are parts the robot has but whose exact
model or count is not recorded anywhere. Check the robot and fill them in. Fastener and
cable counts in particular are **TBC** until someone counts them on the robot.

## 1. Compute and control

| # | Component | Qty | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | NVIDIA Jetson Orin Nano Developer Kit (8 GB, "Super") | 1 | Onboard computer, JetPack 6 / ROS 2 Humble | [NVIDIA](https://marketplace.nvidia.com/en-us/enterprise/robotics-edge/jetson-orin-nano-super-developer-kit/), [Seeed](https://www.seeedstudio.com/NVIDIAr-Jetson-Orintm-Nano-Developer-Kit-p-5617.html) | $249.00 | $249.00 |
| 2 | Storage for the Jetson (microSD or NVMe SSD) — **TBC** | 1 | OS + workspace | any | ~$20–60 | ~$40 |
| 3 | ROBOTIS OpenCR 1.0 | 1 | Motor controller + IMU + ultrasonic input, `/dev/opencr` (USB `0483:5740`) | [ROBOTIS US](https://www.robotis.us/opencr1-0/), [MyBotShop](https://www.mybotshop.de/ROBOTIS-OpenCR10-Dynamixel-Controller_1) | $247.14 | $247.14 |
| 4 | Arduino Nano (ATmega328P, FTDI USB) | 1 | LED controller, `/dev/arduino_nano` (USB `0403:6001`) | [Arduino Store](https://store.arduino.cc/products/arduino-nano) | ~$25 | ~$25 |
| 5 | Adafruit INA219 current/voltage sensor breakout (#904) | 1 | Jetson supply monitoring over I2C (`orin_battery_state`) | [Adafruit](https://www.adafruit.com/product/904) | ~$10 | ~$10 |
| 6 | USB hub (model **TBC**) | 1 | Fans the USB devices (OpenCR, LiDAR, Nano, webcam) out from the Jetson | any powered USB 3 hub | ~$20 | ~$20 |

## 2. Actuators and their frames

The IDs and modes are taken from the motor table in `mecanumbot_microcontrollers`. All motors share
the 1 Mbps Dynamixel bus on the OpenCR. The XMs use protocol 2 and the AXs use protocol 1.
Each servo ships with its horn and horn bolts.

| # | Component | Qty | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 7 | ROBOTIS DYNAMIXEL XM430-W210-T | 4 | Wheel drives, IDs 1–4, velocity mode | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=902-0125-000), [ROBOTIS US](https://www.robotis.us/dynamixel-xm430-w210-t/) | $269.90 | $1,079.60 |
| 8 | ROBOTIS DYNAMIXEL AX-12A | 3 | Neck/camera tilt (ID 7) and the two grabber jaws (IDs 5, 6), position mode | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=902-0003-001), [ROBOTIS US](https://www.robotis.us/dynamixel-ax-12a/) | $49.90 | $149.70 |

### Neck and grabber frames

The frames are light-grey ROBOTIS **FP04** parts, the AX-series (Bioloid / ROBOTIS PREMIUM)
frame family. They are identified by matching the robot photos against the ROBOTIS
product photos, so check them on the robot before ordering. The servo bodies sit in
the printed mounts (rows 38–40). The FP04 frames carry everything from the horn outwards.

| Assembly | Frames, from the horn outwards |
| --- | --- |
| **Neck** (ID 7, tilt) | **FP04-F2** U-bracket over the horn and the back idler → **FP04-F3** flat plate that carries the camera |
| **Grabber jaw** (IDs 5 and 6, one each) | **FP04-F2** U-bracket over the horn and the back idler → **FP04-F3** joiner → **FP04-F11** curved finger |

| # | Component | Pieces needed | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 9 | ROBOTIS FP04-F2 (10 pcs) | 3 (neck 1, jaws 2) | U-bracket on the servo horn | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=903-0036-001), [ROBOTIS US](https://www.robotis.us/fp04-f2-10pcs/) | $7.70 / 10 | $7.70 |
| 10 | ROBOTIS FP04-F3 (10 pcs) | 3 (neck 1, jaws 2) | Camera plate (neck), joiner between the bracket and the finger (jaws). **Verify the neck plate:** F6 looks similar. | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=903-0037-001), [ROBOTIS US](https://www.robotis.us/fp04-f3-10pcs/) | $10.60 / 10 | $10.60 |
| 11 | ROBOTIS FP04-F11 (2 pcs) | 2 | The curved grabber fingers | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=903-0045-001) | $1.60 / 2 | $1.60 |

The FP04 frames are fixed with M2 bolts and nuts from the Bolt Nut Set BNS-10 (row 28).
ROBOTIS sells them without fasteners.

## 3. Sensors

| # | Component | Qty | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 12 | ROBOTIS LDS-02 (LD08) 360° LiDAR, with its USB interface board | 1 | `/scan`, `/dev/ld08_lidar` (USB `10c4:ea60`). **Discontinued in 2025.** The successor is the LDS-03, which needs a different driver and `LDS_MODEL`. | [ROBOTIS shop](https://en.robotis.com/shop_en/list.php?ca_id=4050), [LDS-03 notes](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_03/) | ~$100 | ~$100 |
| 13 | USB (UVC) webcam **with built-in microphone** (model **TBC**) | 1 | `/dev/video0` feeds the DeepStream detectors, and its microphone is the `mecanumbot_audio` input. The configs assume a horizontal FOV of 51°. The Raspberry Pi Camera v2 in `images/mecanumbot.jpg` is the legacy Pi revision. | any UVC webcam with a microphone | ~$30–80 | ~$50 |
| 14 | HC-SR04 ultrasonic distance sensor | 1 | Wired to the OpenCR Arduino header (TRIG pin 2, ECHO pin 3). The firmware reads it as `dms` in `opencr_state` (cm, `-1` on a timeout past about 4 m). `mecanumbot_sensorproc_node` turns it into `has_object`: is something held in the grabber. | [Adafruit #3942](https://www.adafruit.com/product/3942), [SparkFun SEN-15569](https://www.sparkfun.com/products/15569) | ~$4 | ~$4 |

## 4. Power

| # | Component | Qty | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 15 | ROBOTIS LiPo 11.1 V 1800 mAh LB-012 (3S) | 1–2 | Motor/OpenCR supply. The low-battery alarm fires at 9.7 V. | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=903-0210-000), [ROBOTIS US](https://www.robotis.us/lipo-battery-11-1v-1800mah-lb-012/) | $59.60 | $59.60 |
| 16 | Jetson power source (separate battery / DC-DC) — **TBC** | 1 | Supplies the Orin Nano. The INA219 in row 5 monitors it. | — | — | — |
| 17 | LiPo charger (3S balance) | 1 | Charging | ROBOTIS LBB-040 or any 3S balance charger | ~$30 | ~$30 |

## 5. Chassis and drivetrain

| # | Component | Qty | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 18 | TurtleBot3 Waffle Plate-IPL-01 (8 pcs) | 2–3 sets | Three-deck waffle chassis | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=903-0259-000), [ROBOTIS US](https://www.robotis.us/tb3-waffle-plate-ipl-01-8ea/) | $19.00 | ~$57 |
| 19 | Mecanum wheels, 65 mm (2 left + 2 right) | 1 set | Drive wheels. `wheel.radius = 0.0325 m` in `mecanumbot_IO_node` | [DFRobot](https://www.dfrobot.com/product-2302.html), [Yahboom](https://category.yahboom.net/products/mecanum-wheel) | ~$30–50 / set | ~$40 |

## 6. LEDs

The firmware (`Mecanumbot_Nano_LED/LEDutils.h`) drives **32 WS2812B pixels, GRB order,
on Nano pin D5**, split into four bars of 8. From index 0, the bars are back-right,
back-left, front-right and front-left. Each bar sits under a printed diffuser (row 38).

| # | Component | Qty | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 20 | WS2812B addressable LED strip, 5 V, 60 LEDs/m, cut into 4 × 8-pixel bars (≈ 0.55 m used) | 1 m | Signalling LEDs | [Adafruit NeoPixel strip](https://www.adafruit.com/category/183), or any WS2812B 60/m strip (Amazon, AliExpress) | ~$12 / m | ~$12 |
| 21 | 3-wire LED leads (5 V / GND / DIN) with connectors, e.g. JST-SM 3-pin pigtails | 4 | Bar-to-Nano wiring. The photo shows ribbon leads on each bar. | any | ~$8 / pack | ~$8 |
| 22 | 5 V supply for the LEDs — **TBC** | 1 | See the warning below | — | — | — |

⚠ **LED power.** The firmware sets `BRIGHTNESS 240` and no FastLED power limit, so all 32
pixels on white can draw about **1.8 A at 5 V**. A Nano powered over USB cannot supply that.
Record where the bars actually take their 5 V from (for example OpenCR's 5 V header or a
separate regulator) before building a copy.

## 7. Fasteners, standoffs and spacers

The decks are held apart by the TurtleBot3 **plate supports**, which are female–female M3
hex standoffs (`SUPPORT_HEX_M3_0XL35_FF` and `…_0XL45_FF` in the TB3 CAD). ROBOTIS does not
sell them on their own, but any M3 F–F hex standoff of the same length fits. The counts
depend on this robot's deck layout, so they are **TBC**.

| # | Component | Qty | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 23 | M3 hex standoff, female–female, **35 mm** | **TBC** | Deck spacer (TB3 plate support) | generic aluminium/brass M3 F–F standoffs ([Misumi](https://us.misumi-ec.com/), Amazon, AliExpress) | ~$1 / pc | ~$10 |
| 24 | M3 hex standoff, female–female, **45 mm** | **TBC** | Deck spacer (TB3 plate support) | as above | ~$1 / pc | ~$10 |
| 25 | TB3 PCB Support-IBB-01 (12 pcs) | 1 set | Mounts boards (OpenCR, Jetson, Nano) and parts that don't match the waffle hole pitch | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=903-0261-000), [ROBOTIS US](https://www.robotis.us/tb3-pcb-support-ibb-01-12ea/) | $7.00 | $7.00 |
| 26 | Metric screw assortment with nuts and washers: M2.5 and M3 pan-head, 6–12 mm | 1 kit | Plate-to-plate joints, standoff ends, board mounting. The camera bridge, grabber mounts and diffusers are fixed with pan-head M3 screws and washers (see the photo). | generic (Amazon, AliExpress, local hardware store) | ~$15 | ~$15 |
| 27 | ROBOTIS Wrench Bolt M2.5×4 (200 pcs) | 1 pack (spares) | XM430 horn/frame bolts and the wheel-adapter-to-horn joint | [ROBOTIS US](https://www.robotis.us/wrench-bolt-m2-5-4-200-pcs/) | $19.21 | $19.21 |
| 28 | ROBOTIS Bolt Nut Set BNS-10 (M2/M3 bolts, nuts, spacers) | 1 | Fixes the FP04 frames (rows 9–11) to each other and to the AX-12As | [ROBOTIS US](https://www.robotis.us/bolt-nut-set-bns-10/) | $32.09 | $32.09 |

## 8. Cables

The OpenCR's Dynamixel TTL ports and the XM430s both use the **JST** 3-pin connector. The
AX-12A uses the older **Molex** 3-pin connector, so the AX chain needs a JST↔Molex
"convertible" cable at the OpenCR end.

| # | Component | Qty | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 29 | ROBOTIS Robot Cable-X3P 180 mm (10 pcs) | 1 pack | OpenCR ↔ XM430 and XM430 ↔ XM430 daisy chain (JST–JST) | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=903-0249-000), [ROBOTIS US](https://www.robotis.us/robot-cable-x3p-180mm-10pcs/) | $19.00 | $19.00 |
| 30 | ROBOTIS Robot Cable-X3P (Convertible) 180 mm (10 pcs) | 1 pack | OpenCR (JST) → first AX-12A (Molex) | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=903-0251-000), [ROBOTIS US](https://www.robotis.us/robot-cable-x3p-180mm-convertible-10pcs/) | $15.50 | $15.50 |
| 31 | ROBOTIS Robot Cable-3P 180 mm (10 pcs) | 1 pack | AX-12A ↔ AX-12A daisy chain (Molex–Molex): neck and both grabber jaws | [ROBOTIS](https://en.robotis.com/shop_en/list.php?ca_id=302090), [ROBOTIS US](https://robotis.us/robot-cable-3p-180mm-10pcs/) | $16.70 | $16.70 |
| 32 | USB cables: micro-USB (OpenCR), mini-USB (Arduino Nano), the LiDAR interface lead, a webcam extension, and the hub's upstream cable | ~5 | Every serial device reaches the Jetson over USB, through the hub (row 6) | any, short 15–30 cm leads | ~$3 each | ~$15 |
| 33 | Battery lead: LB-012 → OpenCR battery input (extension / connector) — **TBC** | 1 | Motor power | comes with the TurtleBot3 kit, or built in-house | ~$5 | ~$5 |
| 34 | Jetson DC power lead (5.5 × 2.5 mm barrel) — **TBC** | 1 | Jetson power, through the INA219 shunt | depends on row 16 | ~$5 | ~$5 |
| 35 | Dupont jumper wires (F–F) | 1 pack | HC-SR04 → OpenCR header (VCC, TRIG, ECHO, GND) and INA219 → Jetson I2C header (SDA, SCL, 3V3, GND) | any | ~$5 | ~$5 |
| 36 | Cable ties, spiral wrap, heat-shrink | — | Routing along the standoffs and to the neck (see the photo) | any | ~$8 | ~$8 |

## 9. 3D-printed and in-house parts (CUSTOM)

The STLs are in `mecanumbot_description/meshes/3Dparts/`. Price means filament cost only.

| # | Component | Qty | File | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 37 | Camera bridge (the white arch holding the neck servo and camera) | 1 | `cam_bridge.stl` | **CUSTOM** (3D printed) | ~$3 | ~$3 |
| 38 | LED diffusers | 4 | `diffuser.stl` | **CUSTOM** (3D printed) | ~$0.50 | ~$2 |
| 39 | Grabber mount, front (holds the AX-12A bodies) | 1 | `grabbermount_front.stl` | **CUSTOM** (3D printed) | ~$1 | ~$1 |
| 40 | Grabber mount, rear | 1 | `grabbermount_rear.stl` | **CUSTOM** (3D printed) | ~$1 | ~$1 |
| 41 | Wheel adapter (XM430 horn → mecanum wheel hub) | 4 | `omniwheel_adapter.stl` | **CUSTOM** (3D printed) | ~$0.50 | ~$2 |
| 42 | HC-SR04 holder — **TBC** (no STL in the repo) | 1 | — | **CUSTOM** | — | — |
| 43 | LED bar, INA219 and HC-SR04 wiring harness (soldered joints, splices) | 1 | — | **CUSTOM** (built in-house from rows 21 and 35) | — | — |

## 10. Operator side (not mounted on the robot)

| # | Component | Qty | Role | Source | Unit price |
| --- | --- | --- | --- | --- | --- |
| 44 | Gamepad: Sony DualShock 4 **or** Xbox 360 wireless controller + wireless adapter | 1 | `mecanumbot_joy` teleop (udev rules shipped for both) | any electronics retailer | ~$30–60 |
| 45 | ROBOTIS RC-100 remote (optional) | 1 | Standalone OpenCR control (`OpenCR_Core` firmware), not used by the ROS stack | [ROBOTIS shop](https://en.robotis.com/shop_en/) | ~$20 |

## Approximate total

| Group | Approx. |
| --- | --- |
| Compute and control | ~$590 |
| Actuators and frames | ~$1,250 |
| Sensors | ~$155 |
| Power (excluding the Jetson supply) | ~$90 |
| Chassis and drivetrain | ~$100 |
| LEDs (excluding their 5 V supply) | ~$20 |
| Fasteners and standoffs | ~$95 |
| Cables | ~$90 |
| Custom parts | ~$10 |
| **Robot total** | **~$2,395** |

The four XM430 wheel motors are about 45 % of the cost. The
[`Mecanumbot_OpenCR_Stepper`](https://github.com/Fortuz/mecanumbot_microcontrollers)
firmware variant replaces them with NEMA17 steppers on TMC2209 drivers, which costs much
less but adds weight and loses the wheel telemetry. Its README lists the trade-offs.

Buying a complete **TurtleBot3 Waffle Pi** kit (about $1,680, from
[ROBOTIS US](https://www.robotis.us/turtlebot-3-waffle-pi-rpi4-4gb-us/)) covers rows 3,
12, 15, 17, 18, 23–26, 29 and 33 plus 2 of the 4 XM430s. The kit's Raspberry Pi and Pi
camera are then left over.
