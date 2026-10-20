# Crazyflie 2.1 Brushless (crazyflie_bl)

A JSBSim model of Bitcraze's Crazyflie 2.1 Brushless nano quadrotor, set up as it flies in the IDSIA nano-drone
system-identification benchmark (Busetto et al., arXiv 2512.14450): 45 g with a Flow deck v2, an AI deck and a
350 mAh 1S battery, propeller guards, four 08028-10000KV motors on 55-35 propellers, 100 mm motor to motor. Its
thrust, drag, motor and inertia values were identified on that benchmark's training flights. GPL-3.0 for the
flight model; `NOTICE` names every source and `meshes/NOTICE` the meshes' terms.

```
crazyflie_bl.xml   main file; no body aerodynamics
Metrics.xml        reference values only, there is no wing
Mass.xml           45 g, CG trim, inertia
Gear.xml           four feet: the tips of the guards' posts under the motors
Propulsion.xml     M1..M4 in Bitcraze's numbering, hubs 12 mm above the CG
Engines/           bl08028 (brushless DC motor), prop5535 (thrust, torque and in-plane force tables)
Systems/           esc: commanded duty -> voltage across the motor, with the ESC's losses
Battery.xml        1S 350 mAh LiPo pack (sidecar)
Batteries/         LiPo: the cell's resting volts by charge (sidecar)
initGrnd.xml       standing at the benchmark's site, Lugano
Controls.xml       autopilot outputs -> motors, PX4 and ArduCopter rows (sidecar)
Sensors.xml        imu / barometer / gps (sidecar)
Visual.xml         meshes/: airframe with guards, two propellers, chase camera (sidecar)
firmwares/         px4_crazyflie_bl (PX4 airframe 22101)
LICENSE, NOTICE    GPL-3.0, sources
```

## Run it

PX4: `firmwares/px4_crazyflie_bl` is airframe 22101; install it as a SITL airframe and start with
`PX4_SYS_AUTOSTART=22101`. ArduCopter takes the motors in this model's order with `FRAME_CLASS 1` and
`FRAME_TYPE 14` (X, numbered clockwise from front right); no parameter file ships for it.

The propeller file uses three FGPropeller features that JSBSim 1.3 does not have: the `C_INPLANE_FORCE` and
`CT_GROUND_FACTOR` tables, and a rotor inertia below 1e-6 slug ft² taken as given. JSBSim 1.3 reports the tables as
unknown and flies without rotor drag or ground effect, and it raises the inertia to 1.36e-6 kg m², 22 times this
rotor's, so the motors take about a second to respond instead of 43 ms. The ESC reads each motor's
`supply-volts`, which JSBSim 1.3's brushless motor does not have.

## Frames

Structural frame: origin at the centre of the four motor axes in the circuit-board plane, X aft, Y right, Z up,
metres. Bitcraze's body frame is x forward, y left, z up, and the comments in `Propulsion.xml` give each motor in
it. M1 is front right, M2 rear right, M3 rear left, M4 front left; M1 and M3 turn counter-clockwise seen from
above. The throttle is the ESC's duty. `Systems/esc.xml` turns it into the voltage across the motor over the
supply, and PteroSim's pack (`Battery.xml`) writes that supply, its terminal voltage, into each motor's
`supply-volts` every step; without a pack the supply stays at 4.2 V. The ESC also writes `esc-input-duty`, the duty,
so the pack draws that share of each motor's current, the ESC's own loss included.

## Where the numbers come from

| group | value | source |
|---|---|---|
| mass | 45 g | IDSIA paper, total with battery and decks |
| layout | 35.355 mm per axis; numbering and spin | Bitcraze firmware: `ARM_LENGTH` 0.050 m (`platform_defaults_cf21bl.h`), mixer (`power_distribution_quadrotor.c`) |
| propeller | 55 mm, 2 blades | Bitcraze 55-35 |
| thrust | kF 3.726e-8 N/(rad/s)², thrust loss with climb 2.65e-6 N/(rad/s·m/s), as `C_THRUST(J)` | identified |
| torque | kM = `THRUST2TORQUE` 0.0056928 m × kF, as `C_POWER` | Bitcraze firmware |
| rotor drag | kd 2.749e-6 N/(rad/s·m/s), as `C_INPLANE_FORCE` = 0.0870 μ | identified |
| ground effect | thrust × 1/(1 − (R/4z)²) against z/R, as `CT_GROUND_FACTOR`; standing on its feet z/R ≈ 0.96, +7.6 % | Cheeseman and Bennett 1955, single rotor |
| motor | winding 0.52 Ω | Bitcraze 08028-10000KV page |
| motor | Kv 12610 rpm/V, I0 0, rotor and propeller inertia 5.36e-8 kg m² | identified (datasheet: 10000 rpm/V, 0.40 A) |
| ESC | losses as one series resistance, 0.586 Ω | identified |
| battery | 1S 350 mAh LiPo, PteroSim's LiPo voltage curve | IDSIA paper |
| battery | cell resistance 64.5 mΩ | identified |
| battery | avionics 2.07 W: board 0.96 W, AI deck 1.11 W, flow deck not counted | Bitcraze firmware docs (`pwm-to-thrust.md`), AI-deck 1.1 datasheet |
| battery | low warning at 15 % | PX4 `BAT_LOW_THR` default |
| inertia | 3.23e-5, 3.79e-5, 5.46e-5 kg m² | identified |
| CG | 0.123 mm aft and 0.162 mm left of the motor centre; height in the board plane | identified trim; height chosen |
| heights | hubs 12.0 mm above the CG, feet 15.43 mm below | Crazyflow's cf21B meshes, as its `cf21B_500.xml` places them |
| feet | 110 N/m, 1.56 N s/m, friction 0.8 / 0.5 | chosen: 1 mm static deflection, damping ratio 0.7 |

Identified means fitted on the benchmark's training flights (chirp, random, square) to the IMU accelerations and
body rates, the rotor speeds, the duty cycle and the battery voltage. The propeller coefficients are referred to
the standard-atmosphere density at the benchmark's site, 1.19323 kg/m³ at 273 m.

What follows from them: on a full pack hover takes 0.548 duty, 1721 rad/s per rotor, the pack at 4.05 V, and
0.604 at half charge; full duty on a full pack gives 0.225 N (23 g) per motor in still air, the pack sagging to
3.73 V, a thrust-to-weight ratio of 2.0; the rotors' time constant at hover is 43 ms.

## Known limitations

- The rotor drag acts at the hubs. On the benchmark's flights the moments that come with it fit a line of action
  50 mm (forward force) and 36 mm (side force) above the CG, because the blades' flapping adds a moment of its own;
  at the hubs, 12 mm up, that moment is missing, and the pitch and roll moments of flying forward and sideways are
  a quarter and a third of the real vehicle's.
- About a third of the roll and pitch torque in the flight data is not explained by the model, and the CG trim
  moves by about 0.4 mm between sessions. Without a controller its roll and pitch rates part from the real ones
  sooner than a constant-rate guess does.
- No body drag, vertical included.
- The ground effect is the single-rotor Cheeseman-Bennett curve on thrust only; the four rotors' interaction near the
  ground and the lower power in ground effect are not modelled.
- Above about 2.5 V of supply the motor and ESC fall behind Bitcraze's thrust-stand fit in the firmware (2573
  against 2919 rad/s at 4 V), so the top of the thrust range is lower than the real vehicle's. The identified Kv is
  26 % over the datasheet's, and the no-load current sits at zero: the three-constant motor behind an ideal
  duty-cycle ESC does not take the datasheet's values and the flights together.
- The pack is PteroSim's generic LiPo curve at the nominal 350 mAh. In the benchmark's flights the measured
  voltage falls faster through a flight than this pack's.
- The decks are in the mass but not drawn.
