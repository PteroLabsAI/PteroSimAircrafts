# Typhoon H480 (typhoon_h480)

Yuneec's Typhoon H is a 1.95 kg camera hexacopter with retractable skids and the CGO3+ 4K camera on a three-axis
gimbal. This is a JSBSim model of it built from PX4's Gazebo Classic `typhoon_h480`, meshes included (BSD; `NOTICE`
names every source; Yuneec does not endorse PteroSim).

```
typhoon_h480.xml  main file
Metrics.xml       reference values; structural frame = the source model's base link, x aft, y right, z up
Mass.xml          1.145 kg airframe, the 0.550 kg POWER 4 pack and the 0.255 kg CGO3+: 1.950 kg at takeoff
Gear.xml          the four rubber pads at the ends of the skids, retracted with them
Propulsion.xml    six motors in PX4's hexarotor x order
Aero.xml          rotor drag
Engines/          h480_motor (brushless motor), h480_prop (thrust and power tables)
Systems/          esc_map.xml: command to motor duty; skid_retract.xml: the skids follow the gear command
Battery.xml       the POWER 4 pack (sidecar); Batteries/LiPo.xml is its cell curve
Controls.xml      PX4 outputs to the motors and the skids (sidecar)
Payloads.xml      the CGO3+ gimbal and its PX4 outputs (sidecar)
Sensors.xml       imu, gps, barometer and the CGO3+ camera (sidecar)
Visual.xml        meshes (sidecar)
firmwares/        px4_typhoon_h480: the PX4 airframe; run_px4_sitl.sh starts PX4 SITL with it
LICENSE.PX4, NOTICE  BSD 3-Clause for the PX4 material, sources
```

## Run it

In PteroSim, add `typhoon_h480`, choose PX4, set Physics Hz to 250 or more (the IMU reports at 250 Hz) and press Start.
Then start PX4 SITL in WSL from a built PX4 tree, with the Windows host's address:

```bash
export PX4_SIM_HOSTNAME="$(ip route | awk '/default/ {print $3; exit}')"
bash firmwares/run_px4_sitl.sh "$HOME/Documents/PX4-Autopilot/build/px4_sitl_default"
```

The script copies PX4's ROMFS to a temporary folder, installs the airframe under the first free number from 22100 and
starts PX4 on it. Once the estimator is ready, take off from QGroundControl or with `commander takeoff`
([PX4 SITL with PteroSim](https://docs.px4.io/main/en/sim_pterosim/index)).

PX4's outputs 1-6 drive the motors, 7-9 the CGO3's roll, pitch and yaw, and 10 the skids. PX4 raises the skids above
2 m in its auto modes (takeoff, hold, mission, return) and lowers them to land; with no autopilot they stay down.

## What is modelled

- Mass: Yuneec gives 1.950 kg at takeoff and 0.255 kg for the CGO3+. The installed pack's 0.550 kg is estimated from a
  teardown of the stock pack; the inertia is scaled from the source model's links.
- Propellers: the thrust coefficient is the source model's, 8.55 N a rotor at 1000 rad/s. Shaft power and reaction
  torque come from APC's 10x4.5MR data near the hover speed.
- Battery: the POWER 4 is 4S, 14.8 V, 5400 mAh. PteroSim drains it by the motors' current and the avionics, sags its
  voltage under load and sends it to PX4.
- Skids: they swing up 55 degrees in 1.18 s on PX4's gear output, and their pads leave the ground with them.
- Camera: the CGO3+ sees 115 degrees across, on a gimbal turning roll ±45, pitch ±90 and yaw ±180 degrees.

## Known limits

- The airframe has rotor drag only; the body's own drag is not modelled.
- The ESC is a static map set to 1000 rad/s at the full command; its speed loop is not modelled.
- Yuneec publishes the gimbal's rates, the skids' travel and the pack's resistance nowhere, so those values are chosen.
