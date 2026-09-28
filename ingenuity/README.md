# Ingenuity Mars Helicopter (ingenuity)

A JSBSim model of NASA's Ingenuity, the coaxial helicopter that flew on Mars, with NASA's own 3D model as its meshes
(GPL-3.0 for the model files, NASA/JPL-Caltech for the meshes; `NOTICE` names every source). Two counter-rotating
two-bladed rotors of 1.21 m on one mast, each with its own swashplate, four legs. Set up for ArduPilot (ArduCopter,
dual-helicopter frame in coaxial mode). It is meant for a Mars scene; the planet is the scene's setting, not the
vehicle's.

```
ingenuity.xml        main config, includes everything below
Metrics.xml          reference area = the rotor disk, AERORP at the CG
Mass.xml             1.8 kg, CG, inertias
Aero.xml             body drag only
Propulsion.xml       two electric motors, two rotors (engines rotor_upper / rotor_lower)
Gear.xml             four feet
Engines/             ingenuity_motor (electric), ingenuity_upper_rotor / ingenuity_lower_rotor (FGRotor)
Systems/             rotor_control.xml: six swashplate servos -> collective, longitudinal and lateral of each rotor;
                     ingenuity_governor.xml: the RSC command sets the rotor speed, a PID per motor holds it;
                     ingenuity_manual.xml: bank, pitch, yaw rate and lever -> the six servos, for flying by hand
Controls.xml         ArduCopter heli-dual outputs -> JSBSim bindings, plus the manual channels (sidecar)
fly_ingenuity.py     fly it by keyboard or gamepad, or let it fly a climb-hold-land profile
Sensors.xml          imu / barometer / gps (sidecar)
Visual.xml           meshes/: body (fixture), the two rotors (propellers), chase camera (sidecar)
firmwares/           ardupilot_ingenuity.param: self-contained ArduCopter heli-dual SITL set
LICENSE, NOTICE      GPL-3.0, sources
```

## Run it

Under ArduPilot:

```
sim_vehicle.py -v ArduCopter -f heli-dual --model JSON:<host> --add-param-file=firmwares/ardupilot_ingenuity.param
```

By hand (spawn it, start the simulation, then; needs the `pterosim` SDK and `pygame`):

```
python fly_ingenuity.py --input keyboard
python fly_ingenuity.py --input gamepad
python fly_ingenuity.py --auto          # rotors up, climbs, holds, lands, rotors off
```

| command | key | gamepad |
| --- | --- | --- |
| collective lever up / down | `W` / `S` | left stick Y (position) |
| yaw: nose left / right | `A` / `D` | left stick X |
| pitch: nose down / up | `Up` / `Down` | right stick Y |
| bank left / right | `Left` / `Right` | right stick X |
| rotors on (spool up) / off | `Enter` / `Backspace` | buttons 3 / 1 (pygame numbering: Y / B on an Xbox pad) |
| stop | `Esc` | button 6 (Back on an Xbox pad) |

The lever holds where it is left; bank, pitch and yaw spring back. The model's stability system holds the commanded
attitude (up to 20 deg of bank or pitch, 90 deg/s of yaw), so the sticks fly angles, not servos. The lever's bottom
is the rotor's idle, just under zero thrust; it lifts off with the lever a little above 0.15 on Earth and 0.75 on Mars.

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

- The rotor hubs are within 5 cm of the CG and the "upper" one below the "lower"; NASA's model has them 0.453 and
  0.354 m above the feet. Visual.xml draws them where NASA has them.
- The collective ranges disagree: 0..25 deg of root pitch here against `H_COL_ANG_MAX 15` in the param file.
- The motor is far more than hover at Mars density takes; the governor holds the rpm regardless.
- FGRotor's momentum-theory inflow is singular at zero thrust with no flow through the disc: a rotor held at the
  zero-thrust pitch (12.5 deg root) on the ground and then moved runs away (Earth: NaN within 2 s; Mars: an overspeed
  the governor cannot brake). The idle therefore sits at 12.2 deg, -3.7 N on Earth and -0.1 N on Mars, and the
  collective crosses zero thrust moving, which the model survives at any rate up to a step.
- Everything the NOTICE lists as chosen: inertias, CG height, blade moments, lift-curve slope, hinge offset, motor
  power, leg stiffness and damping, drag, governor gains.
