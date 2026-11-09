# Stork (stork)

A six-seat air taxi: a JSBSim model of NASA's Quadrotor reference vehicle in its turboshaft version (Quad TS). Four
three-blade articulated rotors turn on one cross-shafted drive, fed by two 305 hp turboshafts through a combiner
gearbox; the only control is each rotor's collective. It stands on skids. The meshes are built from NASA's OpenVSP
model and come in four liveries (GPL-3.0 for the model files; `NOTICE` and `meshes/NOTICE` name every source; NASA
does not endorse PteroSim).

```
stork.xml       main file; collective stops and pitch-flap coupling (FCS)
Metrics.xml     the four discs as reference area; AERORP = the centre of the drag areas
Mass.xml        empty 2355 lb + six 200 lb seats; inertia
Gear.xml        four skid contacts
Propulsion.xml  the combiner gearbox on rotor 1, the two turboshafts on rotors 2 and 3, a placeholder on rotor 4;
                179 lb tank
Aero.xml        airframe drag (NASA's D/q); no lift or moment
Engines/        stork_turboshaft (305 hp MRP, 319 hp CRP), stork_combiner, stork_shaft, stork_rotor_master and
                stork_rotor_slave (FGRotor)
Systems/        stork_drive.xml: rotor rpm governor, load sharing between the turbines, one-engine-out rating,
                torque gauges
Controls.xml    ArduCopter quad X outputs -> JSBSim bindings (sidecar)
Sensors.xml     imu / barometer / gps (sidecar)
Visual.xml      meshes/<livery>/: body (fixture), four rotors (propellers), chase camera (sidecar)
firmwares/      ardupilot_stork.param: ArduCopter quad SITL set
LICENSE, NOTICE GPL-3.0, sources
```

## Liveries

`meshes/` holds four full sets: `taxi` (the default), `rescue`, `liner` and `police`. To change the livery, change
the folder name in the five `file=` lines of `Visual.xml`.

## Run it

Under ArduPilot, with PteroSim's physics at 1 kHz (at 240 Hz ArduCopter's gyro and loop-rate checks refuse to arm):

```
sim_vehicle.py -v ArduCopter -f quad --model JSON:<host> --add-param-file=firmwares/ardupilot_stork.param
```

The turbines run governed from boot. SERVO5 (function 136) holds the drive's governor engaged whether the vehicle is
armed or not, so the rotors reach speed at flat pitch before the EKF lets it arm, and keep turning after it disarms.
ArduCopter cannot shut the engines down.

The parameter file tunes the route for passengers: 15 m/s cruise, about 10 deg of tilt or less through turns and
stops, and a 0.5 m/s descent in LAND. LAND reads its height above home, so to land on a roof, fly to a
waypoint 10 m above the pad, then NAV_LAND.

## Control mapping (ArduCopter quad X, FRAME_CLASS 1, FRAME_TYPE 1)

channel = SERVOn - 1:

| SERVO | function | channel | JSBSim |
|---|---|---|---|
| SERVO1 | 33 Motor1 | 0 | `propulsion/engine[0]/collective-ctrl-rad`, front right, 0.1175..0.5 rad |
| SERVO2 | 34 Motor2 | 1 | `propulsion/engine[3]/collective-ctrl-rad`, rear left |
| SERVO3 | 35 Motor3 | 2 | `propulsion/engine[1]/collective-ctrl-rad`, front left |
| SERVO4 | 36 Motor4 | 3 | `propulsion/engine[2]/collective-ctrl-rad`, rear right |
| SERVO5 | 136 Max | 4 | engine 0 throttle: above 0.5 the governor holds the rotors at 573.4 rpm |

Each motor output is that rotor's collective, the concept's only control: there is no cyclic and no rpm control.
`Controls.xml` gives the measured signs and the hover settings; the parameter file's header gives the plant and how
the gains were derived.

## Frames

Structural frame: X aft, Y right, Z up, feet, origin at the Center-Of-Mass marker of NASA's OpenVSP file. Rotors 1
to 4 are front right, front left, rear right and rear left; the rear pair sits 3.2 ft higher than the front. The
CG at design gross weight is at (-0.083, 0, -1.532) ft.

## Sources

| tag | source | used for |
|---|---|---|
| S18 | Silva, Johnson, Antcliff, Patterson, AIAA 2018-3847, Table 3, column Quad TS | weights, rotor radius, tip speed, solidity, drag areas, power, the performance the model is checked against |
| J18 | Johnson, Silva, Solis, "Concept Vehicles for VTOL Air Taxi Operations", AHS 2018, NTRS 20180003381 | rotor twist, pitch-flap coupling |
| L6 | NASA's NDARC files for the concept: quadrotor6.list, quadrotor6.run, gen500.list | blade count, lift-curve slope, flap frequency, Lock number, hub and gear drag, the GEN500 engine |
| NT | Johnson, NDARC theory, NASA/TP-2015-218751 | hinge offset and blade inertia, engine scaling, lapse and part-power fuel flow, the one-engine-out rating |
| VSP | NASA's OpenVSP model quadcollflap.vsp3 | all geometry, the mass parts, the fuel tank |

## Results

Pure JSBSim at the design gross weight of 3735 lb, 6000 ft ISA unless noted. Hover and cruise trim to the same
numbers at dt 1/240 and 1/1000 s.

| case | model | NASA [S18] |
|---|---|---|
| hover power, out of ground effect | 289.5 hp, aircraft FM 0.699 | 291.8 hp, FM 0.69 |
| hover fuel flow | 189.3 lb/h | 189.2 lb/h (GEN500 in L6) |
| cruise at 122 KTAS | 267.1 hp, L/De 5.22 | 285.1 hp, L/De 4.9 |
| maximum speed at 90 % MCP | 141.7 KTAS | 142.0 KTAS |
| best endurance speed | 60.5 KTAS | 58.0 KTAS |
| best range speed | 102.4 KTAS | 122.0 KTAS |
| autorotation index KE/P | 0.958 s | 0.95 s |
| engine spool-up, 15 % to 95 % power | 2.95 s | no number; 5 s at most by 14 CFR 33.73(b) |
| one engine out in the hover | sinks at 11.7 ft/s, the other engine at its contingency power | one 305 hp engine is short of the hover |
| one engine out in a sea-level hover | holds; the rotors droop to 546.5 rpm (95 %) | no number |

## Known limits

- The best range speed comes out at 102 kt against NASA's 122, and cruise power 6 % under NASA's: the power curve
  is flat within 2 % from 90 to 125 kt, because FGRotor has neither NDARC's induced-power factor nor the front
  rotors' wake on the rear ones.
- FGRotor has no stall or compressibility. Above about 146 kt the model enters a limit cycle.
- Yaw comes only from the rotors' torque, so it is weak in cruise. A hard GUIDED stop from above about 115 kt
  saturates yaw: the heading swings and the vehicle balloons before it recovers.
- The CG from the VSP file's mass parts sits 0.72 ft aft of the CG NDARC flies its performance cases at.
- Ground effect is off (`groundeffectexp` 0).
- The engines' N1, ITT and start values are template values: NASA publishes no such data for the GEN500.
- No cabin interior: the windows are opaque.
- Only the turboshaft version. NASA's electric Quadrotor is not modelled.
