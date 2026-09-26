# eXplora Tailsitter VTOL (explora)

A JSBSim model of the [eXplora Tailsitter VTOL](https://github.com/robustini/eXploraVTOL), the open-hardware
3D-printed tailsitter by Marco Robustini and his team (GPL-3.0; `NOTICE` names the team and every source).

Dual-motor flying wing: two tractor motors on the wing, two elevons, a fin above and below the fuselage, stands
on its tail on three rods. Flies on ArduPlane as a QuadPlane tailsitter and on PX4.

```
explora.xml          main config, includes everything below
Metrics.xml          reference dimensions, AERORP at the AVL neutral point
Mass.xml             weight, CG, inertias
Aero.xml             AVL derivatives; the wing as two propeller-washed strips and the free-stream rest
FlightControl.xml    two elevon servos -> elevator / aileron
Gear.xml             3 tail-sitting feet + crash contacts
Propulsion.xml       2 motors, thrust along body +X
Engines/             T-Motor MN4012 480KV + 14x8 propeller (QPROP tables)
Controls.xml         autopilot outputs -> JSBSim bindings (ArduPlane and PX4 rows)
Sensors.xml          imu / baro / gps / airspeed
Airframe.xml         rests pitch 90, sensors in the plane frame
Visual.xml           meshes/: airframe + motors, elevons, propellers, chase camera
firmwares/           ardupilot_explora.param (ArduPlane SITL), px4_explora (PX4 airframe 22006)
LICENSE, NOTICE      GPL-3.0, sources
```

## Run it

ArduPlane, with the team's parameter file translated to current names (the file's header says what was left out):

```
sim_vehicle.py -v ArduPlane --model JSON:<host> --add-param-file=firmwares/ardupilot_explora.param
```

PX4: `firmwares/px4_explora` is airframe 22006; install it as a SITL airframe and start with `PX4_SYS_AUTOSTART=22006`.

## Frames

Body X runs along the fuselage; the wing's +3 deg incidence is inside the alpha tables of `Aero.xml`, not a
mount angle. Hover is theta 90, nose up: `Airframe.xml` rests the aircraft at pitch 90 and leaves the sensors in
the plane frame, as ArduPlane expects for a tailsitter (level is calibrated in fixed-wing flight). The motors
point along the fuselage axis.

```
CAD (STEP assembly):   x = span toward the LEFT wing, y up, z forward, mm
Aircraft frame:        origin nose tip, X forward, Y right, Z DOWN, m
                       X = (z - 534.8)/1000   Y = -x/1000   Z = -(y - 0.9)/1000
JSBSim structural:     origin nose tip, X AFT, Y right, Z UP, m
```

## Control mapping (ArduPlane)

From the team's parameter file, channel = SERVOn - 1:

| SERVO | function | channel | JSBSim |
|---|---|---|---|
| SERVO9 | 73 ThrottleLeft | 8 | motor 0, `motor_left` |
| SERVO10 | 74 ThrottleRight | 9 | motor 1, `motor_right` |
| SERVO5 | 77 ElevonLeft | 4 | `fcs/servo-0-cmd-norm`, +1 = trailing edge up |
| SERVO6 | 78 ElevonRight, SERVO6_REVERSED 1 | 5 | `fcs/servo-1-cmd-norm`, +1 = trailing edge down |

The elevons are bound per surface because ArduPlane emits the mixed elevon outputs (`ArduPlane/servos.cpp`:
left = elevator - aileron, right = elevator + aileron, non-reversed servos go up); SERVO6_REVERSED flips the
right channel for the mirrored linkage. Elevon throw is +-45 deg per side.

## Provenance

Geometry is measured on the upstream STEP files, masses come from the 3MF slicer projects and the BOM, the
aerodynamics from AVL and the propeller tables from QPROP; every number in the files names its source. Three
values are not measured on the real aircraft and are marked "chosen" where they appear: the CG height, the
elevon throw and weathervaning (off in the parameter file). `NOTICE` lists the sources and tools.
