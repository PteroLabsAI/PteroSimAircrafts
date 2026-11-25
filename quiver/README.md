# Quiver (quiver)

Arrow Air's Quiver Dev-Kit is a 25 kg MTOW quadcopter with four Hobbywing X6 Plus G2 motors, MFP 24x8.0 folding props
and a 14S Tattu 4.0 pack. This model carries the 30 Ah long-endurance pack (21.4 kg at take-off). It is a JSBSim model
built from Arrow Air's open hardware repository, meshes included (CERN-OHL-S-2.0; `NOTICE` names every source; Arrow
Air does not endorse PteroSim).

```
quiver.xml      main file
Metrics.xml     reference areas; structural frame = the CAD plate-stack centre, x aft, y right, z up
Mass.xml        empty 9.65 kg with CG and inertia from the CAD parts; the 30 Ah pack, 11.40 kg
Gear.xml        the four foam pads under the skids
Propulsion.xml  four motors at the CAD motor centres, props at their hub plane
Aero.xml        body drag per axis
Engines/        X6PlusG2 (electric motor), MFP2480 (thrust and power tables, in-plane force)
Systems/        esc_map.xml: throttle command to motor voltage, from Hobbywing's 54 V table
Battery.xml     the pack (sidecar): Tattu 4.0 14S 30 Ah; Batteries/LiHV.xml is its cell curve
Controls.xml    ArduCopter outputs to the motors and the gimbal (sidecar)
Sensors.xml     imu, gps, barometer, temperature, the gimbal camera, streamed (sidecar)
Visual.xml      meshes and the SIYI A8 mini gimbal (sidecar)
firmwares/      ardupilot_quiver.param: Arrow's ArduCopter parameters, set up for SITL
LICENSE, NOTICE CERN-OHL-S-2.0, sources
```

## Run it

Under ArduCopter SITL, with PteroSim's physics at 1 kHz:

```
sim_vehicle.py -v ArduCopter -f quad --model JSON:<host> --add-param-file=firmwares/ardupilot_quiver.param
```

The param file keeps Arrow's battery failsafe (RTL at BATT_LOW_VOLT 48 V, land at BATT_CRT_VOLT 44.8 V) and its
fence (30 m up, 60 m out). ArduPilot reads the pack from the JSON state as an analog monitor (BATT_MONITOR 4). Before
arming, PteroSim also sends the pack's charge, so a part-charged start shows its real percentage. The start charge is
`initial_charge_percent` in `Battery.xml`.

The model needs PteroLabs' JSBSim fork (PteroLabsAI/jsbsim) for two things: the prop's in-plane force table and the
motors' `supply-volts`. Stock JSBSim logs "Unknown table type" for the in-plane table and applies no in-plane force.

## Known limits

- The ESC is a static map; its own speed loop is not modelled.
- Apart from the props' in-plane force, the airframe has body drag only: no download and no edgewise inflow.
- The A8 mini's field of view and the gimbal's rates are defaults, not SIYI's figures. Pointed straight down, its
  camera sees the landing gear.
