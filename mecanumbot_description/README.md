# Mecanumbot description package

**Function:** contains data files needed to operate the mecanumbot

## Data folders

#### maps:

Contains .yaml descriptions and .pgm map files (occupancy grids) for rviz map loading.

---

#### meshes:

Stereolitography files (.stl) of robot parts, important for simulations (#TODO)

---

#### param:

Contains .yaml files with parameters to customize node operations

---

#### udev:

Udev rules (*.rules) are configuration files controlling the response to a given device in the event of its connection (e.g. remapping to port)
These need to be placed in either **/usr/lib/udev/rules.d** or **/usr/local/lib/udev/rules.d** to be applicable.

To apply new rules, place them to the correct folder then

```
$ sudo udevadm control --reload-rules # apply new rules
$ sudo udevadm trigger #set rules to device
```

---

#### urdf:

Contains URDF (Unified Robot Description Format ) files, which depict the robot. The most important usecase is simulations and visualisations (needed for state_publishers).
