# eXplora Tailsitter VTOL — JSBSim model (explora)

A simulation model of the [eXplora Tailsitter VTOL](https://github.com/robustini/eXploraVTOL), the open-source
3D-printed tailsitter by Marco Robustini and team (GNU GPL v3.0; `NOTICE` names every contributor and source).

Dual-motor tailsitter flying wing: two tractor motors on the wing, two elevons, two fixed fins above and below
the fuselage, stands on its tail on three rods. Flown by ArduPlane in QuadPlane tailsitter mode (Q_TAILSIT_ENABLE 1,
no thrust vectoring). Structure follows the PteroSimAircrafts layout (one file per component, `Controls.xml` /
`Sensors.xml` / `Visual.xml` / `Airframe.xml` sidecars).

```
explora.xml          main config, includes everything below
Metrics.xml          reference dimensions, AERORP at the AVL neutral point
Mass.xml             weight, CG, inertias
Aero.xml             AVL derivatives + wing partition: two propeller-washed strips and the free-stream rest
FlightControl.xml    two elevon servos -> elevator / aileron for Aero.xml (ArduPlane does the mixing)
Gear.xml             3 tail-sitting feet + crash contacts
Propulsion.xml       2 motors, thrust along body +X
Engines/             T-Motor MN4012 480KV + 14x8 propeller (QPROP tables)
Controls.xml         ArduPlane output -> JSBSim bindings (sidecar)
Sensors.xml          imu / baro / gps / airspeed (sidecar)
Airframe.xml         rests pitch 90, sensors in the plane frame (sidecar)
Visual.xml           meshes/: airframe + motors (fixtures), elevons (left/right-aileron), propellers, chase camera (sidecar)
firmwares/           ardupilot_explora.param: self-contained ArduPlane SITL set from the real dump (header says what was not copied);
                     px4_explora: PX4 airframe 22006, dual-motor elevon tailsitter
LICENSE, NOTICE      GPL-3.0, upstream design files
```

Every number in the files names its source; a number with no source is marked "chosen". The model flies ArduPlane
and PX4 SITL (section SITL). Three numbers are chosen so that it does: the CG height, the elevon throw and
weathervaning off. Nothing is calibrated against a real flight.

## Frames — read this first

**Body X runs along the FUSELAGE axis.** The wing sits on it at +3.0 deg incidence (2.90..3.02 deg LE-up at every
STEP station), carried inside the alpha tables of `Aero.xml`, not as a mount angle.

```
CAD (STEP assembly):   x = span toward the LEFT wing, y up, z forward, mm
Aircraft frame:        origin nose tip, X forward, Y right, Z DOWN, m
                       X = (z - 534.8)/1000   Y = -x/1000   Z = -(y - 0.9)/1000
JSBSim structural:     origin nose tip, X AFT, Y right, Z UP, m
                       x = -X_ac   y = Y_ac   z = -Z_ac
```

Consequences:

- Hover = theta 90, nose up. `Airframe.xml` rests it at pitch 90 and leaves the sensors in the plane frame:
  ArduPlane flies a tailsitter with the autopilot "level" in fixed-wing flight (ardupilot.org
  guide-tailsitter: "The AHRS_ORIENTATION, the accelerometer calibration and Level trim should all be done for
  fixed wing flight"). The PX4 `quadtailsitter` needs `<sensors pitch="90"/>`; this vehicle must not copy it.
- Thrusters use `pitch = 0` (shafts along the fuselage axis: the Motor_Support bores run along CAD +z and its
  back face is flush on Wing_L3's front face, cant 0, toe 0).
- The three feet are 552 mm aft of the nose tip; the CG is 289 mm aft of it, so the CG stands 263 mm above the
  ground.

## Control mapping (ArduPlane)

From the real aircraft's parameter file (upstream `parameters/Ardupilot/2022-10-17_Pixhawk1.param`),
channel = SERVOn - 1:

| SERVO | function | channel | JSBSim |
|---|---|---|---|
| SERVO9 | 73 ThrottleLeft | 8 | Motor index 0, `motor_left` |
| SERVO10 | 74 ThrottleRight | 9 | Motor index 1, `motor_right` |
| SERVO5 | 77 ElevonLeft | 4 | `fcs/servo-0-cmd-norm`, +1 = trailing edge UP |
| SERVO6 | 78 ElevonRight, SERVO6_REVERSED 1 | 5 | `fcs/servo-1-cmd-norm`, +1 = trailing edge DOWN |

The elevons are bound per surface (the way `standard_vtol` binds its ArduPlane ailerons), not as
Elevator/Aileron, because ArduPlane emits the mixed elevon outputs. The sign comes from
`ArduPlane/servos.cpp`: `channel_function_mixer(k_aileron, k_elevator, k_elevon_left, k_elevon_right)` computes
left = elevator - aileron, right = elevator + aileron, with the comment "the order is setup so that non-reversed
servos go 'up'". SERVO6_REVERSED flips the right channel for the mirrored right-hand linkage; the sim receives
the flipped value, so +1 on it means trailing edge down. `FlightControl.xml` turns the two into
`fcs/elevator-pos-rad` (> 0 both TE down, nose down) and `fcs/aileron-pos-rad` (> 0 left TE down, right TE up,
roll right), which `Aero.xml` consumes. SERVO3/4 = 76/75 TiltMotorRight/Left are assigned in the param but
Q_TAILSIT_VFGAIN = VHGAIN = 0, so they never leave trim; not bound.

Elevon throw: **+-45 deg per side, chosen so it flies**. No source gives the servo/linkage travel; with +-30 deg the
ArduPlane back-transition stalled at 30..60 deg tilt and crashed in SITL. The printed cove allows 63 deg TE down /
65.5 deg TE up (STEP clash limit).

## What the model does

Loads clean in **JSBSim 1.3.1** (`python -c "jsbsim.FGFDMExec(...).load_model('explora')"`), no warnings, both
engines recognised. Probed at 240 Hz:

- all-up mass 3.400 kg, CG x 0.2888 / z 0.0171 m, on the thrust line (matches `Metrics.xml` EYEPOINT and VRP);
- FCS signs as in the table above, all six sign cases pass;
- full throttle held down at theta 90: 42.0 N per motor at 9272 rpm (22.2 V), the propulsion force is along
  body +X, the counter-rotating senses cancel the motor torque to 0.000 N m;
- on the ground the CG is 18.1 mm on the upper-surface side of the wing-cap line, so the model rests on the two caps
  and the upper fin ball's crash contact, 1.0 deg off vertical (PteroSim: status pitch 89.0, not crashed). The drop
  test that stood it on all three feet (WOW 1/1/1) was run with the earlier belly-side CG.

Static and trim numbers: static T/W 2.52 at 3.4 kg (50.6 A per motor at full throttle, the BOM's ESC rating),
hover throttle 0.612 (thrust fraction 0.42 in ArduPilot's mixer terms), -0.046 N m per degree of elevon in hover
(the washed elevon force is capped at the jets' momentum flux, 2 T delta; the linear AVL law gave twice that), the
washed wing strips lift 5.1 N (15 % of the weight) toward the upper surface in still-air hover, +1.2 deg standing
elevon trim (the thrust line passes through the CG; what is left is the strips' own moment), 10 % differential
thrust rolls toward the weaker motor through the strips, pure spanwise wind leaves the alpha tables and beta
derivatives at zero and gives 5.2 N of broadside drag with a 0.37 N m weathercock moment about the CG, 16 m/s trim
at alpha 4.8 deg / elevator -0.8 deg / throttle 0.40, Vs 11.55 m/s, short period 8.0 rad/s zeta 0.42. The hover authority is a bound, not a
validated number. The visual turning signs are derived, not seen (open question 5).

## SITL

PteroSim standalone game, lockstep at 4x, 2026-09-25. ArduPlane runs `firmwares/ardupilot_explora.param`, PX4 runs
airframe 22006 (`firmwares/px4_explora`).

| stack | test | result |
|---|---|---|
| none | standing on the feet: IMU, attitude, GPS, airspeed | 9/9 |
| ArduPlane | QLOITER hover at 15 m, land | 8/9: tilt RMS 10.0 deg against a 5 deg limit; drift 0.6 m, spin 0.16 deg/s |
| ArduPlane | AUTO: climb, transition, 16 m/s cruise, back-transition, VTOL land | 7/8: transition 0.7 s, cruise 15.8 m/s at 42 % throttle; the back-transition ends by timeout; lands 0.37 m from home |
| PX4 | hover at 15 m, land | 8/9: tilt RMS 9.8 deg; drift 0.3 m |
| PX4 | mission, same profile | 8/8: transition 1.4 s, cruise 16.4 m/s at 42 % throttle, lands 0.21 m from home |

The hover failure on both stacks is the model's steady ~10 deg lean (open question 10).

## Model notes

**Wing partition.** `Aero.xml` does not treat "the elevon in the wash" as a special case. The planform is three parts
with the same coefficient tables (the segmented-wing slipstream model of Khan & Nahon, ICUAS 2015): a washed strip
behind each propeller (the contracted wake, y = 0.325 +- 0.130 m, 0.0794 m2 each) and the free-stream rest. A strip
sees u + k v_i of its own motor (k 1.878, JSBSim's momentum-theory `prop-induced-velocity`, vortex-tube value 1.84 R
behind the disc), its own alpha and dynamic pressure, and produces X, Z, a pitch moment (Cm plus its quarter-chord
arm), roll and yaw at +-0.322 m. With the props stopped the three parts add up to the whole-wing AVL model exactly.
What it adds: the wash normal force of the 3 deg-incidence wing in hover (5.1 N, 15 % of the weight, toward the
upper surface, plus Cm0), the differential-thrust roll coupling, the CZq wash force, and a transition corridor that
follows from the same geometry. Each elevon is 0.623 inside its strip by area, the rest in the free stream; the
elevon terms act on dynamic-pressure-weighted deflections.

**Washed elevon force capped at the jets' momentum flux.** In still air a flap in a jet cannot push harder than the
jet momentum it turns, T sin delta; the linear AVL derivative on the full wash q asked the two jets for 2.0x the
momentum they carry (11.95 N for 10 deg against 2 x 2.96 N). The wash increment of each strip's chordwise elevon q
is `min(1/2 rho max(u_s, 0)^2 - q_free, C T_engine)`, C = eta / (|CZ_de| S phi/2) = 0.4113 psf per lbf, so the
washed elevon force per side is at most eta T delta. eta = 1.0 is the momentum-theory ceiling, chosen; NACA TN 3307
is reported to give ~0.6 for a 90 %-chord double flap at static thrust, a different flap, not verified here. The
free-stream part is untouched (cruise trim elevator -0.77 deg). Consequences: hover pitch authority -0.0464 N m/deg
(-0.88 rad/s2 per deg), roll 0.145 N m/deg, standing elevon trim +1.2 deg (3 % of the chosen throw), implied
ArduPlane rate-loop crossover with the shipped gains (Q_A_RAT_PIT_P 0.17, Q_A_RAT_YAW_P 0.4) over the +-45 deg
throw 6.8 rad/s (pitch) and 10.6 (body roll). Nothing here validates the authority, it bounds it (open question 10).

**Sideslip beyond small angles.** The alpha tables, CD0 and the q-rate terms act on `explora/qbar-xz-psf` = qbar
cos^2 beta; Y_beta, Roll_beta and Yaw_beta act on sin beta cos beta; a crossflow term (`explora/crossflow/side-force-lbs`,
-CD 1/2 rho v|v| A_side with CD 1.2 chosen and A_side = fuselage side projection 0.0428 m2 + fins 0.0291 m2) feeds
`Y_crossflow` and a `Yaw_crossflow` written 57 mm aft of the AERORP (70 mm aft of the CG once JSBSim adds the
12.7 mm AERORP-to-CG arm; the arm must not be written from the CG or it is counted twice). In 10 m/s pure spanwise
flow at hover: side force -5.2 N, yaw +0.37 N m nose into the wind.

**Stall speed against the param.** Vs is 11.55 m/s at 3.4 kg against AIRSPEED_MIN 12. ArduPlane's `ARSPD_FBW_MIN`
description says "Should be set to 20% higher than level flight stall speed", which would put Vs at 10 m/s; meeting
that needs CLmax 1.34 where the tables carry 1.005 (AVL's linear value at body alpha 12 deg cut at the cited NACA
0012 stall). The STEP section argues against more: the root, mid and tip sections have 2 % camber at 30 % c with
the camber line going to -0.06 % c at 90 % c, a reflexed trailing edge; AVL puts the zero-lift line at section
alpha -0.1 deg (CL0 0.210 at body 0) and Cm0 at +0.0146 nose-up. A reflexed 11 % section at Re 3.2e5 is not
expected to reach the CLmax of a plain-camber one, so 1.0 was not raised. The same param with the 5.1 kg MTOM
configuration would need CLmax 1.9 to meet the rule, so it was not set that way.

**Battery voltage.** `maxvolts` 22.2 V = 6S nominal, the voltage of every QPROP hover/validation row and the
SwanK1_Motor.xml convention; JSBSim has no battery sag. Supply/ESC resistance is not folded into `coilresistance`:
the Turnigy page lists no internal resistance and the ~0.2 Ohm implied by the T-Motor bench is that rig's supply.

## Data provenance

### Upstream design files (GPL-3.0)

`robustini/eXploraVTOL`, commit `1da97a4172505b71aa9e709a2d7ab6e27e3a97a8` (2026-06-07, "Set Q_TAILSIT_VHGAIN
to 0"). Used: `STEP/eXplora_VTOL_V1.00_*.step` (36 parts, placed into one assembly by mating features), `3MF/`
slicer projects (shell/infill for the printed masses), `resources/BOM.md`, `resources/explora_vtol_000.jpg` /
`_001.jpg` (Xoar 14x8 stamp, resting posture, livery), `parameters/Ardupilot/2022-10-17_Pixhawk1.param`,
`README.md` (right wing = mirrored left).

### Measured on the STEP files (OpenCASCADE)

| Quantity | Value | Feature |
|---|---|---|
| Span | 1.3682 m (short caps), 1.5218 m (Wing_L5_Long) | Wing_L4 plane x=660 + cap bbox |
| Root / tip chord | 337.1 / 218.6 mm | sections at x_cad 122 / 659.5 |
| Gross area / MAC | 0.4061 m2 / 0.3032 m | planform integral to the centreline |
| Wing incidence | +3.0 deg, uniform, no washout | LE-TE line at 21 stations |
| t/c | 0.110 root .. 0.128 tip | sections |
| Elevons | x_cad 121..539, chord 120 mm, hinge 0.67..0.59 c | Elevon_L1/L2 + hinge bore r 5.2 |
| Fins | 24 deg rake, 0.0266 m2 both | Fuselage_4 raked bore, streamwise slices |
| Fuselage | 549 mm long, 196 x 111 mm | Fuselage_1..5 + Canopy_1 |
| Motor axis | X_ac -0.1938 (support face), Y +-0.325, Z_ac -0.0171 | Motor_Support shaft bore |
| Feet | wing caps (664.5, -0.1, -17.0) mm, fin ball lowest z -17.4 | tube caps, ball cap |
| Nose tip | CAD (0, 0.9, 534.8) mm | Fuselage_1 max-z vertex |

### Computed

| Quantity | Value | How |
|---|---|---|
| Masses, CG, inertias | 3.4 kg, CG x 0.2888 z 0.0171; Mass.xml Ixx 0.243 Iyy 0.043 Izz 0.280 (empty x 1.337, about the empty CG); all-up 0.245 / 0.053 / 0.288 | STEP solids x 3MF slicer settings, datasheet point masses, tubes, battery, 0.686 kg unaccounted lump placed to put the CG on the thrust line, with an airframe-like inertia share (both chosen) |
| Neutral point, derivatives | Xnp 0.3015 m (23.1 % MAC), static margin +4.2 % | AVL 3.52, 4 surfaces, 1128 vortices, no fuselage body |
| CD0 | 0.0282 | wetted-area build-up |
| Wing partition (hover elevon effectiveness, wash lift) | two washed strips y 0.325 +- 0.130 m, 0.0794 m2 each, k 1.88; each elevon 0.623 inside its strip by area | momentum theory + vortex-tube contraction on the STEP chord stations, `Aero.xml` header |
| Propeller Ct/Cp(J) | Ct0 0.0899, zero thrust J 0.759 | QPROP 1.22, anchored on UIUC apce_14x12 and the T-Motor bench |
| Crossflow drag (beta -> 90) | CD 1.2 x (fuselage side 0.0428 + fins 0.0291 m2), centre 70 mm aft of the CG (written 57 mm aft of the AERORP; JSBSim adds the rest) | STEP fuselage stations + fin area; CD chosen (Hoerner ch. III) |
| Washed elevon ceiling | wash increment of a strip's elevon q capped at 0.4113 psf per lbf of its engine's thrust, so the washed elevon force per side is at most T delta (eta 1.0, momentum theory) | the turning efficiency is chosen at the ceiling |
| Gear spring / damping | 6800 N/m, 227 N/m/s | quadtailsitter's 3000 / 100 (1.5 kg) x 3.4 / 1.5; drop-tested with the earlier belly-side CG |

### From datasheets and documents

| | |
|---|---|
| T-Motor MN4012 KV480 | KV 480, 57 mOhm, 1.3 A idle @ 10 V, 44.7 x 32.5 mm, 155 g (store.tmotor.com spec table) |
| Battery | Turnigy 5000 mAh 6S 680.4 g (hobbyking.com); BATT_CAPACITY 5000 (param); maxvolts 22.2 V = 6S nominal (no sag modelled) |
| Propeller | Xoar 14x8 beech (photo stamp), 25 g (Xoar PJN 14 in) |
| All-up mass | 3.4 kg without payload, 5.1 kg MTOM with wing extensions (marco3dr, ArduPilot Discuss 92341, 2024-04-27) |
| ArduPlane behaviour | tailsitter guide (sensor frame), `ArduPlane/servos.cpp` (elevon sign), `AP_Baro_MS5611.cpp` (baro rate) |
| Sensor rates | imu 300 Hz = SCHED_LOOP_RATE, gps 5 Hz = GPS_RATE_MS 200, baro 80 Hz = MS5611 driver (100 Hz timer, 4 of 5 slots pressure) |

### Chosen, no evidence

- Motor rotation senses (left +1, right -1). Counter-rotating is certain (the ArduPlane dual-motor tailsitter
  layout), which side turns which way is not; only the residual roll torque sign depends on it.
- Elevon throw +-45 deg, so that the SITL back-transition completes (see Control mapping).
- The 0.686 kg unaccounted lump: placed so the all-up CG lies on the thrust line (z 0.0171). At 60 mm on the belly
  side, the standing photo's side, the thrust line was 22.4 mm above the CG and the model tipped over at throttle-up
  in SITL. Its inertia share equal to 0.337 x the empty tensor (its candidates, wiring, LEDs,
  screws, slicer shortfall, are all distributed).
- Crossflow drag coefficient 1.2 for the fuselage side projection and the fins (Hoerner ch. III flat plate / cylinder).
- Jet turning efficiency eta 1.0 (the momentum-theory ceiling) for the washed elevon force.
- Crash-contact spring/damping = the feet's; prop hub thickness 8 mm (prop plane at x 0.153).
- `Q_WVANE_ENABLE 0` in `firmwares/ardupilot_explora.param`, where the real aircraft flies 3: weathervaning reads the
  model's steady hover lean as wind, rocked the hover left-right and spiralled the landing (PX4: `WV_EN 0`).
- Sensors.xml carries no camera (none in the BOM) and no pitot position (Pitot_cover has no mating feature).

### Weak evidence, cross-check only

The RealFlight `eXplora_VTOL.rfvehicle` (chord 0.36/0.245, NACA 0014, placeholder masses) agrees with the STEP
to 4..11 % on chords and on the elevon extent; it was not used for any number. The "s=0.45 b=1.88" figures in the
forum thread are ArduPilot's skywalker_2013 SITL defaults, not eXplora.

## Open questions

1. **CG height is a choice, not a measurement.** The all-up CG z 0.0171 is where the model hovers: on the thrust
   line. The standing photo says the real CG is on the belly side of the wing-cap line (z -0.001), at least 18 mm
   lower. Placed there (z -0.0053) the model tipped over at throttle-up. Either the real CG is higher than the photo
   suggests, or the real aircraft carries the thrust-line moment with authority the model lacks (open question 10).
   A two-point weighing of the real aircraft would settle it.
2. **Elevon throw.** Only the geometric clash limit (63 / 65.5 deg) is known; the servo horn and pushrod are not
   in the STEP set and SERVO5/6 MIN/MAX 1000/2000 say nothing about degrees. +-45 deg is chosen because +-30 deg
   crashed the back-transition in SITL. The hover pitch gain scales with it directly. The washed elevon cap is
   linear, eta T delta: at full throw it gives 0.785 T per side where turning the jet through 45 deg gives at most
   T sin 45 = 0.707 T, 11 % over the momentum bound (4.7 % at 30 deg).
3. **Prop plane X.** The motor body and prop hub are not in the STEP set; 0.153 m = support face + 16 mm motor
   half-height (MN4012 32.5 mm) + 8 mm hub (chosen). A 10 mm error moves the hover thrust line's pitch arm by
   nothing (the arm is in z) and the slipstream distance to the elevons by 4 %.
4. **Throttle against the param.** Hover: JSBSim needs duty 0.612 at 22.2 V (thrust fraction 0.42 in ArduPilot's
   mixer terms; `firmwares/ardupilot_explora.param` starts there) where the real aircraft learned Q_M_THST_HOVER
   0.235, inconsistent with 3.4 kg on two 14x8 props by 1.65x; do not tune the propulsion to it. Cruise: 0.40 at
   16 m/s against TRIM_THROTTLE 0.30. Both are linear PWM duty in forward flight (`Tailsitter::output` ->
   `output_motor_mask`, no expo), so the gap is physical: the prop needs J < 0.759 (its zero-thrust advance ratio
   from an assumed chord/polar, a real 14x8 with more effective pitch would need less rpm) and TRIM_THROTTLE is
   only the TECS feed-forward (ArduPlane default 45, lowered by the builder).
5. **Visual signs.** The mesh frame follows the loader's glTF basis and was checked against `c172x`'s meshes (file z
   runs nose to tail, file x is the span), so the airframe cannot land tail-forward or mirrored. Derived from the
   runtime's quaternion convention and not yet seen in the editor: the elevon hinge axis sign (`0 -1 0`, positive
   reported angle = trailing edge down) and each propeller's spin axis (`motor_left`, sense +1, about `-1 0 0`).
   `c172x` writes the opposite hinge sign, so one of the two files is wrong; a look in PIE settles it. The
   fuselage payload cut-out ahead of the canopy has no cover placed (`Fuselage_Payload_Holder` and `FC_Housing`
   have no mating feature) and shows as an opening.
6. **Sideslip between the limits.** `Aero.xml` is exact at beta 0 (AVL) and beta 90 (broadside drag) and a smooth
   blend between; nothing measured constrains beta 20..70 deg, the lateral rate terms (p, r) keep the full qbar at any
   beta, and no crossflow roll term is written (the fins are near-symmetric about the fuselage axis); what does exist is
   the roll JSBSim makes by applying the side force at the AERORP, 5.9 mm below the CG: +0.031 N m at 10 m/s spanwise
   (0.2 deg of aileron at the capped hover authority), rolling the belly away from the wind.
7. **Rotation senses, propeller polar, CD0, stall shoulder**: see "Chosen"; the QPROP table is expected 5..12 %
   low on thrust and 10..23 % low on power against the T-Motor bench (`Engines/explora_prop_14x8.xml` header). A
   section polar (XFOIL on the STEP section) is the only evidence that could move CLmax either way.
8. **Wing_L5_Long.** The 5.1 kg MTOM configuration (long tips, span 1.5218 m, gross area 0.4295 m2) is measured
   but not modelled; Metrics/Aero are the short-tip aircraft.
9. **The transition corridor is derived, not calibrated.** The washed strips see the undeflected jet plus the
   crossflow: a 4 m/s crossflow across the 15.5 m/s jet puts them at 14 deg and 160 Pa, 26 N of normal force; at
   8..12 m/s they sit at 19..38 deg on the Viterna post-stall curve. One k (1.878, the elevon mid-chord value) is
   used over the whole strip chord where the vortex tube gives 1.36 at the leading edge, the jet is not bent by the
   crossflow, and AVL's wing lift slope stands in for a blown 0.26 m strip. A slipstream survey or a tethered
   hover-in-wind test of the real aircraft is the evidence that would calibrate it; nothing in the upstream
   repository does.
10. **The trim elevon's side force and the jet turning efficiency.** With the CG on the thrust line the still-air
    standing trim is +1.2 deg, so nothing balances the washed strips' lift (5.1 N toward the upper surface): SITL
    hovers leaning ~10 deg on both stacks, and that force alone gives atan(5.1 / 33.3) = 8.6 deg. In a 10 m/s
    headwind the model balances at 38 deg tilt with -23.7 deg of elevon and 16.6 N of elevon force against 23.3 N of
    thrust, where turning the jets through the flap angle could give at most 9.4 N: inside the throw, outside the
    momentum bound, while the author reports flying in more than 10 m/s. The suspects are the elevon throw, the CG
    height (open questions 1 and 2), the wash factor k and the post-stall centre of pressure of the free-stream wing.
    Whether the real aircraft hovers leaning ~10 deg in still air is the cheapest question to ask the author; a
    tethered hover test measuring pitch acceleration per degree of elevon is the evidence that would set eta.
