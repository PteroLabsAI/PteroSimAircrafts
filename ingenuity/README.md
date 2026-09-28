# Ingenuity Mars Helicopter (ingenuity)

A JSBSim model of NASA's Ingenuity, the coaxial helicopter that flew on Mars, with NASA's own 3D model as its meshes
(GPL-3.0 for the model files, NASA/JPL-Caltech for the meshes; `NOTICE` names every source). Two counter-rotating
two-bladed rotors of 1.21 m on one mast, each with its own swashplate, four legs. Set up for ArduPilot (ArduCopter,
dual-helicopter frame in coaxial mode); as it stands it does not hold roll, see the limits below. It is meant for a
Mars scene; the planet is the scene's setting, not the vehicle's.

```
ingenuity.xml        main config, includes everything below
Metrics.xml          reference area = the rotor disk, AERORP at the CG
Mass.xml             1.8 kg, CG, inertias
Aero.xml             body drag only
Propulsion.xml       two electric motors, two rotors (engines rotor_upper / rotor_lower), a placeholder battery tank
Gear.xml             four feet + crash contacts
Engines/             ingenuity_motor (electric), ingenuity_upper_rotor / ingenuity_lower_rotor (FGRotor)
Systems/             rotor_control.xml: six swashplate servos -> collective, longitudinal and lateral of each rotor
Controls.xml         ArduCopter heli-dual outputs -> JSBSim bindings (sidecar)
Sensors.xml          imu / barometer / gps (sidecar)
Visual.xml           meshes/: body (fixture), the two rotors (propellers), chase camera (sidecar)
firmwares/           ardupilot_ingenuity.param: self-contained ArduCopter heli-dual SITL set
LICENSE, NOTICE      GPL-3.0, sources
```

## Run it

```
sim_vehicle.py -v ArduCopter -f heli-dual --model JSON:<host> --add-param-file=firmwares/ardupilot_ingenuity.param
```

## Frames

Body X is the direction the colour camera under the fuselage looks, Z up in the structural frame, the CG at the
origin. The rotors sit on the mast above the CG; the feet stand 0.13 m below it.

## Control mapping (ArduCopter heli-dual, H_DUAL_MODE 2)

channel = SERVOn - 1, the functions heli-dual assigns by default:

| SERVO | function | channel | JSBSim |
|---|---|---|---|
| SERVO1..3 | 33..35 Motor1..3 (swashplate 1) | 0..2 | `fcs/upper-swash-servo0..2` |
| SERVO4..6 | 36..38 Motor4..6 (swashplate 2) | 3..5 | `fcs/lower-swash-servo0..2` |
| SERVO8 | 31 HeliRSC | 7 | both motors' throttle |

ArduPilot emits the H3-120 servo positions (servos at -60, +60 and 180 deg); `Systems/rotor_control.xml` demixes each
plate's three into collective, longitudinal and lateral cyclic for its rotor. Yaw is differential collective
(`H_DCP_SCALER -0.25`, `H_YAW_SCALER 0`).

## Known limits

- The lower rotor's lateral cyclic is inverted in `Systems/rotor_control.xml`. ArduPilot's coaxial mixer sends the
  same roll to both swashplates, so the two rotors tilt against each other and roll cannot be held.
- The rotor hubs are within 5 cm of the CG and the "upper" one below the "lower"; NASA's model has them 0.453 and
  0.354 m above the feet. Visual.xml draws them where NASA has them.
- The 1 lb "fuel" tank makes the vehicle 2.25 kg; Ingenuity is 1.8 kg.
- The collective ranges disagree: 0..25 deg of root pitch here against `H_COL_ANG_MAX 15` in the param file.
- There is no rotor speed governor, and the motor is far more than hover at Mars density takes.
- The leg damping is too high for the vehicle's inertia at the simulator's physics rate: a disturbance on the
  ground leaves a standing pitch-rate oscillation.
- Everything the NOTICE lists as chosen: inertias, CG height, blade moments, lift-curve slope, hinge offset, motor
  power, leg stiffness, drag.
