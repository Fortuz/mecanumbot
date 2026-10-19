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

**TurtleBot3 parts.** The marker in the *Component* column shows which parts come in a
standard **TurtleBot3 Waffle Pi** kit:

- 🟦 the kit supplies the part in full.
- 🟨 the kit supplies some of it: 2 of the 4 XM430s, and 1 of the 2 LiPo batteries.
- No marker: the part has to be bought separately.

The totals at the end show the robot's price with and without these parts.

## 1. Compute and control

| # | Component | Qty | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | NVIDIA Jetson Orin Nano Developer Kit (8 GB, "Super") | 1 | Onboard computer, JetPack 6 / ROS 2 Humble | [NVIDIA](https://marketplace.nvidia.com/en-us/enterprise/robotics-edge/jetson-orin-nano-super-developer-kit/), [Seeed](https://www.seeedstudio.com/NVIDIAr-Jetson-Orintm-Nano-Developer-Kit-p-5617.html) | $249.00 | $249.00 |
| 2 | Storage for the Jetson (microSD or NVMe SSD) — **TBC** | 1 | OS + workspace | any | ~$20–60 | ~$40 |
| 3 | 🟦 ROBOTIS OpenCR 1.0 | 1 | Motor controller + IMU + ultrasonic input, `/dev/opencr` (USB `0483:5740`) | [ROBOTIS US](https://www.robotis.us/opencr1-0/), [MyBotShop](https://www.mybotshop.de/ROBOTIS-OpenCR10-Dynamixel-Controller_1) | $247.14 | $247.14 |
| 4 | Arduino Nano (ATmega328P, FTDI USB) | 1 | LED controller, `/dev/arduino_nano` (USB `0403:6001`) | [Arduino Store](https://store.arduino.cc/products/arduino-nano) | ~$25 | ~$25 |
| 5 | Adafruit INA219 current/voltage sensor breakout (#904) | 1 | Jetson supply monitoring over I2C (`orin_battery_state`) | [Adafruit](https://www.adafruit.com/product/904) | ~$10 | ~$10 |
| 6 | USB hub (model **TBC**) | 1 | Fans the USB devices (OpenCR, LiDAR, Nano, webcam) out from the Jetson | any powered USB 3 hub | ~$20 | ~$20 |

## 2. Actuators and their frames

The IDs and modes are taken from the motor table in `mecanumbot_microcontrollers`. All motors share
the 1 Mbps Dynamixel bus on the OpenCR. The XMs use protocol 2 and the AXs use protocol 1.
Each servo ships with its horn and horn bolts.

| # | Component | Qty | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 7 | 🟨 ROBOTIS DYNAMIXEL XM430-W210-T | 4 | Wheel drives, IDs 1–4, velocity mode | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=902-0125-000), [ROBOTIS US](https://www.robotis.us/dynamixel-xm430-w210-t/) | $269.90 | $1,079.60 |
| 8 | ROBOTIS DYNAMIXEL AX-12A | 3 | Neck/camera tilt (ID 7) and the two grabber jaws (IDs 5, 6), position mode | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=902-0003-001), [ROBOTIS US](https://www.robotis.us/dynamixel-ax-12a/) | $49.90 | $149.70 |

### Neck and grabber frames

The frames are light-grey ROBOTIS **FP04** parts, the AX-series (Bioloid / ROBOTIS PREMIUM)
frame family. They are identified by matching the robot photos against the ROBOTIS
product photos, so check them on the robot before ordering. The servo bodies sit in
the printed mounts (rows 38–39). The FP04 frames carry everything from the horn outwards.

| Assembly | Frames, from the horn outwards |
| --- | --- |
| **Neck** (ID 7, tilt) | **FP04-F2** U-bracket over the horn and the back idler → **FP04-F3** flat plate that carries the camera |
| **Grabber jaw** (IDs 5 and 6, one each) | **FP04-F2** U-bracket over the horn and the back idler → **FP04-F3** joiner → **FP04-F11** curved finger |

| # | Component | Pieces needed | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 9 | ROBOTIS FP04-F2 (10 pcs) | 3 (neck 1, jaws 2) | U-bracket on the servo horn | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=903-0036-001), [ROBOTIS US](https://www.robotis.us/fp04-f2-10pcs/) | $7.70 / 10 | $7.70 |
| 10 | ROBOTIS FP04-F3 (10 pcs) | 3 (neck 1, jaws 2) | Camera plate (neck), joiner between the bracket and the finger (jaws). **Verify the neck plate:** F6 looks similar. | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=903-0037-001), [ROBOTIS US](https://www.robotis.us/fp04-f3-10pcs/) | $10.60 / 10 | $10.60 |
| 11 | ROBOTIS FP04-F11 (2 pcs) | 2 | The curved grabber fingers | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=903-0045-001) | $1.60 / 2 | $1.60 |

The FP04 frames are fixed with M2 bolts and nuts from the Bolt Nut Set BNS-10 (row 27).
ROBOTIS sells them without fasteners.

## 3. Sensors

| # | Component | Qty | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 12 | 🟦 ROBOTIS LDS-02 (LD08) 360° LiDAR, with its USB interface board | 1 | `/scan`, `/dev/ld08_lidar` (USB `10c4:ea60`). **Discontinued in 2025.** The successor is the LDS-03, which needs a different driver and `LDS_MODEL`. | [ROBOTIS shop](https://en.robotis.com/shop_en/list.php?ca_id=4050), [LDS-03 notes](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_03/) | ~$100 | ~$100 |
| 13 | USB (UVC) webcam **with built-in microphone** (model **TBC**) | 1 | `/dev/video0` feeds the DeepStream detectors, and its microphone is the `mecanumbot_audio` input. The configs assume a horizontal FOV of 51°. The Raspberry Pi Camera v2 in `images/mecanumbot.jpg` is the legacy Pi revision. | any UVC webcam with a microphone | ~$30–80 | ~$50 |
| 14 | HC-SR04 ultrasonic distance sensor | 1 | Wired to the OpenCR Arduino header (TRIG pin 2, ECHO pin 3). The firmware reads it as `dms` in `opencr_state` (cm, `-1` on a timeout past about 4 m). `mecanumbot_sensorproc_node` turns it into `has_object`: is something held in the grabber. | [Adafruit #3942](https://www.adafruit.com/product/3942), [SparkFun SEN-15569](https://www.sparkfun.com/products/15569) | ~$4 | ~$4 |

## 4. Power

| # | Component | Qty | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 15 | 🟨 ROBOTIS LiPo 11.1 V 1800 mAh LB-012 (3S) | 2 | One supplies the motors and the OpenCR. The other supplies the Jetson through a custom cable (row 33) and the INA219 (row 5). The low-battery alarm fires at 9.7 V. The TurtleBot3 kit includes only one. | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=903-0210-000), [ROBOTIS US](https://www.robotis.us/lipo-battery-11-1v-1800mah-lb-012/) | $59.60 | $119.20 |
| 16 | 🟦 LiPo charger (3S balance) | 1 | Charging | ROBOTIS LBB-040 or any 3S balance charger | ~$30 | ~$30 |

## 5. Chassis and drivetrain

| # | Component | Qty | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 17 | 🟦 TurtleBot3 Waffle Plate-IPL-01 (8 pcs) | 2–3 sets | Three-deck waffle chassis | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=903-0259-000), [ROBOTIS US](https://www.robotis.us/tb3-waffle-plate-ipl-01-8ea/) | $19.00 | ~$57 |
| 18 | Mecanum wheels, 65 mm (2 left + 2 right) | 1 set | Drive wheels. `wheel.radius = 0.0325 m` in `mecanumbot_IO_node` | [DFRobot](https://www.dfrobot.com/product-2302.html), [Yahboom](https://category.yahboom.net/products/mecanum-wheel) | ~$30–50 / set | ~$40 |

## 6. LEDs

The firmware (`Mecanumbot_Nano_LED/LEDutils.h`) drives **32 WS2812B pixels, GRB order,
on Nano pin D5**, split into four bars of 8. From index 0, the bars are back-right,
back-left, front-right and front-left. Each bar sits under a printed diffuser (row 37).

The HESTORE price is the single-unit price, 744 Ft including 27 % VAT, or 586 Ft without it,
checked on 2026-10-19. It drops to 502 Ft each from 5 pieces. Dollar figures assume about 335 Ft/$.

| # | Component | Qty | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 19 | CJMCU-2812B-8 WS2812B RGB LED module, 8 LEDs, 5 V | 4 | Signalling LEDs, one module per bar | [HESTORE 100.410.78](https://www.hestore.hu/prod_10041078.html) | 586 Ft (~$1.75) | ~$7 |
| 20 | 3-wire LED leads (5 V / GND / DIN) with connectors, e.g. JST-SM 3-pin pigtails | 4 | Bar-to-Nano wiring. The photo shows ribbon leads on each bar. | any | ~$8 / pack | ~$8 |
| 21 | 5 V supply for the LEDs — **TBC** | 1 | See the warning below | — | — | — |

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
| 22 | 🟦 M3 hex standoff, female–female, **35 mm** | **TBC** | Deck spacer (TB3 plate support) | generic aluminium/brass M3 F–F standoffs ([Misumi](https://us.misumi-ec.com/), Amazon, AliExpress) | ~$1 / pc | ~$10 |
| 23 | 🟦 M3 hex standoff, female–female, **45 mm** | **TBC** | Deck spacer (TB3 plate support) | as above | ~$1 / pc | ~$10 |
| 24 | 🟦 TB3 PCB Support-IBB-01 (12 pcs) | 1 set | Mounts boards (OpenCR, Jetson, Nano) and parts that don't match the waffle hole pitch | [ROBOTIS](https://en.robotis.com/shop_en/item.php?it_id=903-0261-000), [ROBOTIS US](https://www.robotis.us/tb3-pcb-support-ibb-01-12ea/) | $7.00 | $7.00 |
| 25 | 🟦 Metric screw assortment with nuts and washers: M2.5 and M3 pan-head, 6–12 mm | 1 kit | Plate-to-plate joints, standoff ends, board mounting. The camera bridge, grabber mounts and diffusers are fixed with pan-head M3 screws and washers (see the photo). | generic (Amazon, AliExpress, local hardware store) | ~$15 | ~$15 |
| 26 | ROBOTIS Wrench Bolt M2.5×4 (200 pcs) | 1 pack (spares) | XM430 horn/frame bolts and the wheel-adapter-to-horn joint | [ROBOTIS US](https://www.robotis.us/wrench-bolt-m2-5-4-200-pcs/) | $19.21 | $19.21 |
| 27 | ROBOTIS Bolt Nut Set BNS-10 (M2/M3 bolts, nuts, spacers) | 1 | Fixes the FP04 frames (rows 9–11) to each other and to the AX-12As | [ROBOTIS US](https://www.robotis.us/bolt-nut-set-bns-10/) | $32.09 | $32.09 |

## 8. Cables

The OpenCR's Dynamixel TTL ports and the XM430s both use the **JST** 3-pin connector. The
AX-12A uses the older **Molex** 3-pin connector, so the AX chain needs a JST↔Molex
"convertible" cable at the OpenCR end.

The motors use **3 of the OpenCR's Dynamixel ports**:

| OpenCR port | Chain | Cables |
| --- | --- | --- |
| 1 | OpenCR → wheel XM430 → wheel XM430 | 2 × X3P 240 mm (row 28) |
| 2 | OpenCR → wheel XM430 → wheel XM430 | 2 × X3P 240 mm (row 28) |
| 3 | OpenCR → AX-12A → AX-12A → AX-12A (the neck and both jaws) | 1 × X3P convertible 180 mm (row 29), then 2 × 3P 200 mm (row 30) |

⚠ The **Robot Cable-3P-XL** (for example the 160 mm 5-pack) does **not** fit the wheel
motors. ROBOTIS sells it for the XL-320 only, and says it is not compatible with the
other X-series servos. The XM430s need **X3P** cables.

Prices in rows 28–30 are robotis.hu list prices excluding VAT, checked on 2026-10-19.

| # | Component | Qty | Role on the robot | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 28 | ROBOTIS Robot Cable-X3P **240 mm** (10 pcs) | 4 pieces from 1 pack | Wheel motors, as two daisy-chained pairs: OpenCR → XM430 → XM430, twice (JST–JST). At least 20 cm is needed, so the kit's 180 mm X3P cables are too short. 240 mm is the longest X3P length ROBOTIS sells. | [ROBOTIS.hu](https://www.robotis.hu/11-robotis?page=4&limit=100), [ROBOTIS](https://en.robotis.com/shop_en/list.php?ca_id=302090) | $21.40 | $21.40 |
| 29 | ROBOTIS Robot Cable-X3P (Convertible) 180 mm (10 pcs) | 1 piece from 1 pack | OpenCR (JST) → first AX-12A (Molex) | [ROBOTIS.hu](https://www.robotis.hu/11-robotis/489-robot-cable-x3p-convertible-180mm-10pcs), [ROBOTIS US](https://www.robotis.us/robot-cable-x3p-180mm-convertible-10pcs/) | $15.50 | $15.50 |
| 30 | ROBOTIS Robot Cable-3P **200 mm** (10 pcs) | 2 pieces from 1 pack | AX-12A ↔ AX-12A daisy chain (Molex–Molex): the neck and both grabber jaws on one chain | [ROBOTIS.hu](https://www.robotis.hu/11-robotis/500-robot-cable-3p-200mm-10pcs) | $17.20 | $17.20 |
| 31 | USB cables: micro-USB (OpenCR), mini-USB (Arduino Nano), the LiDAR interface lead, a webcam extension, and the hub's upstream cable | ~5 | Every serial device reaches the Jetson over USB, through the hub (row 6) | any, short 15–30 cm leads | ~$3 each | ~$15 |
| 32 | 🟦 Battery lead: LB-012 → OpenCR battery input (extension / connector) — **TBC** | 1 | Motor power | comes with the TurtleBot3 kit, or built in-house | ~$5 | ~$5 |
| 33 | Jetson power cable: LB-012 → INA219 shunt → 5.5 × 2.5 mm barrel plug | 1 | Jetson power from the second LiPo (row 15) | **CUSTOM** (built in-house) | ~$5 | ~$5 |
| 34 | Dupont jumper wires (F–F) | 1 pack | HC-SR04 → OpenCR header (VCC, TRIG, ECHO, GND) and INA219 → Jetson I2C header (SDA, SCL, 3V3, GND) | any | ~$5 | ~$5 |
| 35 | Cable ties, spiral wrap, heat-shrink | — | Routing along the standoffs and to the neck (see the photo) | any | ~$8 | ~$8 |

## 9. 3D-printed and in-house parts (CUSTOM)

The STLs are in `mecanumbot_description/meshes/3Dparts/`. Price means filament cost only.

| # | Component | Qty | File | Source | Unit price | Line total |
| --- | --- | --- | --- | --- | --- | --- |
| 36 | Camera bridge (the white arch holding the neck servo and camera) | 1 | `cam_bridge.stl` | **CUSTOM** (3D printed) | ~$3 | ~$3 |
| 37 | LED diffusers | 4 | `diffuser.stl` | **CUSTOM** (3D printed) | ~$0.50 | ~$2 |
| 38 | Grabber mount, front (holds the AX-12A bodies) | 1 | `grabbermount_front.stl` | **CUSTOM** (3D printed) | ~$1 | ~$1 |
| 39 | Grabber mount, rear | 1 | `grabbermount_rear.stl` | **CUSTOM** (3D printed) | ~$1 | ~$1 |
| 40 | Wheel adapter (XM430 horn → mecanum wheel hub) | 4 | `omniwheel_adapter.stl` | **CUSTOM** (3D printed) | ~$0.50 | ~$2 |
| 41 | HC-SR04 holder — **TBC** (no STL in the repo) | 1 | — | **CUSTOM** | — | — |
| 42 | LED bar, INA219 and HC-SR04 wiring harness (soldered joints, splices) | 1 | — | **CUSTOM** (built in-house from rows 20 and 34) | — | — |

## 10. Operator side (not mounted on the robot)

| # | Component | Qty | Role | Source | Unit price |
| --- | --- | --- | --- | --- | --- |
| 43 | Gamepad: Sony DualShock 4 **or** Xbox 360 wireless controller + wireless adapter | 1 | `mecanumbot_joy` teleop (udev rules shipped for both) | any electronics retailer | ~$30–60 |
| 44 | ROBOTIS RC-100 remote (optional) | 1 | Standalone OpenCR control (`OpenCR_Core` firmware), not used by the ROS stack | [ROBOTIS shop](https://en.robotis.com/shop_en/) | ~$20 |

## Approximate total

These are the sums of the *Line total* columns, so the `~` estimates are included. The
operator-side items in section 10 are left out. *Without TurtleBot3 parts* leaves out the
🟦 rows, two of the four XM430s in row 7 and one of the two batteries in row 15 (both 🟨). So it is what remains to buy for
someone who already has a TurtleBot3 Waffle Pi.

| Group | Total | Without TurtleBot3 parts | TurtleBot3 rows in the group |
| --- | --- | --- | --- |
| Compute and control | ~$591 | ~$344 | 3 |
| Actuators and frames | ~$1,249 | ~$709 | 7 (2 of 4) |
| Sensors | ~$154 | ~$54 | 12 |
| Power | ~$149 | ~$60 | 15 (1 of 2), 16 |
| Chassis and drivetrain | ~$97 | ~$40 | 17 |
| LEDs (excluding their 5 V supply) | ~$15 | ~$15 | — |
| Fasteners and standoffs | ~$93 | ~$51 | 22–25 |
| Cables | ~$92 | ~$87 | 32 |
| Custom parts | ~$9 | ~$9 | — |
| **Robot total** | **~$2,450** | **~$1,369** | |

The TurtleBot3 parts are therefore worth about **$1,080** when bought one by one.

### TurtleBot3 price

| Kit | Price (ROBOTIS US, checked 2026-10-19) |
| --- | --- |
| [TurtleBot3 Waffle Pi, RPi4 4 GB](https://www.robotis.us/turtlebot-3-waffle-pi-rpi4-4gb-us/) | **$1,933.61** |
| [TurtleBot3 Burger, RPi4 4 GB](https://www.robotis.us/turtlebot-3/) (for comparison) | $783.50 |

European resellers charge about €1,880–2,060 for the Waffle Pi including VAT.

**Two ways to build the robot:**

- **Starting from a Waffle Pi kit:** $1,933.61 + ~$1,369 ≈ **~$3,303**.
- **Buying every part separately:** ≈ **~$2,450**.

Buying the parts separately is cheaper because the kit also pays for parts this robot does
not use: the Raspberry Pi 4, the Pi camera, the standard wheels and the caster. The kit is
worth buying only if there is already one in the lab to take apart.

The four XM430 wheel motors are about 45 % of the cost. The
[`Mecanumbot_OpenCR_Stepper`](https://github.com/Fortuz/mecanumbot_microcontrollers)
firmware variant replaces them with NEMA17 steppers on TMC2209 drivers, which costs much
less but adds weight and loses the wheel telemetry. Its README lists the trade-offs.

A complete **TurtleBot3 Waffle Pi** kit covers the 🟦 rows (3, 12, 16, 17, 22–25 and 32), 2 of the 4
XM430s in row 7 and 1 of the 2 batteries in row 15. Its X3P cables are 180 mm, too short
for this robot's wheel chains (row 28). The kit's Raspberry Pi and Pi camera are then left over.
