# Orca (orca)

An unmanned version of NASA's Quiet Single Main Rotor helicopter (QSMR, the 450 ft/s design): one 6-blade articulated
rotor, a turboshaft, and a circulation-control tail boom with a direct-jet thruster for anti-torque (the NOTAR
principle). A JSBSim model written from NASA's published design data, with meshes built from NASA's OpenVSP model
(GPL-3.0 for the model files; `NOTICE` and `meshes/NOTICE` name every source; NASA does not endorse PteroSim).

```
orca.xml        main file; the direct-jet thruster's force (external_reactions)
Metrics.xml     disc reference area; AERORP = the CG at design gross weight
Mass.xml        empty 2526 lb + six 200 lb seats; inertia
Gear.xml        three wheels
Propulsion.xml  one turboshaft driving the rotor (engine main_rotor), shaft 3 deg forward, anticlockwise from above;
                186 lb tank
Aero.xml        rotor-wake geometry at the airframe, drag and download, the circulation-control boom (8 slot segments),
                fins, stabiliser
Engines/        orca_turboshaft (462 hp), orca_main_rotor (FGRotor)
Systems/        notar.xml: fan, boom pressure, slot, thruster, fan power on the rotor brake, shaft load;
                orca_governor.xml: load feed-forward plus a PID on rotor rpm, power demand to N1. notar runs first.
Controls.xml    ArduCopter heli outputs -> JSBSim bindings (sidecar)
Sensors.xml     imu / barometer / gps (sidecar)
Visual.xml      meshes/: body (fixture), thruster cone (surface on fcs/notar/sleeve-turn-deg), rotor (propeller),
                chase camera (sidecar)
firmwares/      ardupilot_orca.param: self-contained ArduCopter heli SITL set
LICENSE, NOTICE GPL-3.0, sources
```

## Run it

Under ArduPilot, with PteroSim's physics at 1 kHz (at 240 Hz ArduCopter's 400 Hz loop and gyro checks refuse to arm):

```
sim_vehicle.py -v ArduCopter -f heli --model JSON:<host> --add-param-file=firmwares/ardupilot_orca.param
```

Arm once the EKF uses GPS; armed earlier, it drifts. The param file sets `TKOFF_SLEW_TIME 5`: with ArduPilot's 2 s
take-off slew the collective pull droops the rotor to 81 %, the anti-torque (which goes as rpm squared) runs out
and the heading swings 34 deg; at 5 s the rotor holds 89 % and the heading 6 deg.

## Control mapping (ArduCopter single heli, H_SW_TYPE 1)

channel = SERVOn - 1, the functions a single heli assigns by default:

| SERVO | function | channel | JSBSim |
|---|---|---|---|
| SERVO1 | 33 Motor1, H1 roll | 0 | `propulsion/engine[0]/lateral-ctrl-rad`, +-0.35 rad |
| SERVO2 | 34 Motor2, H1 pitch | 1 | `propulsion/engine[0]/longitudinal-ctrl-rad`, +-0.35 rad, inverted |
| SERVO3 | 35 Motor3, H1 collective | 2 | `propulsion/engine[0]/collective-ctrl-rad`, 0..0.5 rad |
| SERVO4 | 36 Motor4, tail | 3 | `fcs/notar/pedal-cmd-norm`, -1..1 |
| SERVO8 | 31 HeliRSC | 7 | engine 0 throttle: above 0.5 the governor holds 243.5 rpm |

H1 sends roll, pitch and collective each on its own servo, which is what FGRotor takes, so no demixer is needed.
FGRotor's positive longitudinal cyclic pitches the nose down, ArduPilot's positive pitch is nose up: hence the
inversion. `Controls.xml` and the param file's header give the measured responses and the gains' derivation.
`orca.xml` holds the collective between the blade pitch stops, 6.8 to 28.65 deg root pitch (H_COL_ANG_MIN and MAX):
until the autopilot's first servo packet the collective sits at 0 rad, 8.8 deg under zero thrust, and the idling
rotor's downforce rolled the aircraft backwards on its unbraked wheels.

## Frames

Structural frame: X aft, Y right, Z up, feet, origin at the rotor hub (NASA's OpenVSP frame). The CG at design gross
weight sits on the shaft line, 4.18 ft below the hub: NASA publishes no CG, and the shaft leans 3 deg forward (the VSP
blades' front tip is lower than the back), so the line passes x = 4.18 tan 3 deg = +0.219 ft at that depth. The model
hovers at +3.0 deg pitch with zero cyclic.

## Sources

| tag | source | used for |
|---|---|---|
| J20 | Johnson, "A Quiet Helicopter for Air Taxi Operations", VFS 2020, NTRS 20200000509, Table 2 (450 ft/s) and pp. 5-8 | weights, R, Nb, sigma, twist, hinge, power, sfc, D/q, tank, anti-torque shares, all NASA performance |
| V22 | Ventura Diaz et al., VFS 2022, NTRS 20220006620, Tables 1, 3, 4 | Omega 25.5 rad/s, chords, hover condition |
| VSP | NASA OpenVSP model QEU2023-SMR-6pax-turbo-notar_450fps.vsp3 | all geometry, measured projected areas |
| S21 | Whiteside et al., NASA/TM-20210017971, sizing conditions and Table 12 | 10 lb fixed useful load, hover at 6000 ft, cruise at 10000 ft |
| NT/NI | NDARC theory (2022) and input (2025) manuals | a = 5.7, Coleman gradient, tail stall model, kDL 0.05, 4 % gearbox loss, MRP/MCP 1.27, Cheeseman-Bennett |
| ERF | Logan, Morger, Sampatacos, ERF 1983 paper 67 | constant boom pressure, Vj >= 3.5 x wake, fan efficiency 0.73, the thruster's sleeve valve (p. 67-1) and its pedal schedule (Fig. 4), fan diameter |
| Bakir | Bakir, METU thesis 2008, 2.3.4 and ch. 3; Figs. 3.7-3.8 (Dawson-Thompson) | thruster cones and openings, thruster momentum force, Cmu 0.4 optimum, boom fade data |
| MK | Mechanika 2018, Table 2 (Sampatacos' experiment) | Cy vs Cmu |
| Luo | ICAS 2000 | Cy saturation above Cmu ~0.45 |
| L06 | Leishman, Principles of Helicopter Aerodynamics, 2nd ed. 2006, ch. 3 (BEMT, Prandtl tip loss) and ch. 10 | the hover inflow shape over the disc |
| L72 | Landgrebe, JAHS 17(4) 1972 (as in L06 ch. 10) | tip-vortex contraction at the boom's depth |

Cited: DGW 3912 lb, WE 2516 lb, payload 1200 lb, tank 186 lb, R 17.64 ft, 6 blades, sigma 0.0994, twist -12 deg,
hinge 0.045 R, 2 x 231 hp, sfc 0.574, D/q 5.29 ft2, Omega 25.5 rad/s, a 5.7, fan efficiency 0.73, gearbox loss 4 %,
kDL 0.05, the Cy(Cmu) table, the anti-torque shares, root chord 0.290 m / tip 0.174 m (60 % taper from 0.94 R), root
cutout 0.15 R, Landgrebe's contraction constants.

Measured on the VSP model: hub, shaft tilt 3 deg, wheels, tank, seat boxes, boom (axis z -3.27, D 2 ft), slot
(x 9.29-18.09), jet window, boom end (circle r 1.00 ft centred z -3.173 over x 21.5-24.0, end x 24.51), stabiliser
trailing edge x 21.33, fins (2 x 2.74 x 4.08 ft), stabiliser (2.56 x 8.69 ft), top area under the disc 115.6 ft2,
side area 151.3 ft2.

Derived: chord 0.918 ft (sigma pi R / Nb) and tip loss B 0.979; Pitt-Peters inflow lag 0.129 s; blade I_b 97.0
slug ft2, S 8.64 slug ft, Lock number 12.4, polar moment 719 slug ft2; the empty CG and base inertia, from the DGW
targets; gear springs and damping; vertical drag 48.9 ft2 (CD 0.423) and side drag 59.3 ft2; fin and stabiliser slopes
(Helmbold) and arms; boom pressure 49.0 psf (Vj 203 fps), slot width 0.0233 ft (Cmu 0.4) and neutral jet area
1.709 ft2, both from the segmented boom; fin incidence 0.103 rad (70 % of the anti-torque at cruise); the thruster
cutout's half-angle 49.29 deg = A_n / (L r_m) and its neutral angle 120 deg = 180 - 60; the 189 hp brake limit; the
governor's inverse engine map; the boom's downwash.

Reconciled: DGW - WE - payload = 196 lb, but the tank holds 186. The 10 lb difference is fixed useful load, matching
S21's sister designs (operating weight = empty + 10 lb), so the tank starts full and 10 lb sit in the empty weight.

Chosen, no evidence: the blades' share of the rotor group's mass (0.6), the hub's radius of gyration, the fan's mass
(10 lb) and speed, the CG on the shaft line, gear stiffness, damping and friction, fin and stabiliser CLmax 1.0, the
thruster's seam x 21.6, cone radii 0.97 / 0.75 ft, cutout span x 21.85-24.15 and openings +-60 deg, the cutout twice
the neutral opening, the engine output speed 6016 rpm, the governor gains, 8 slot segments and their self-similar
contraction.

## Direct-jet thruster

A sleeve valve of two concentric truncated cones [ERF p. 67-1; Bakir 2.3.4], in `Systems/notar.xml`. The fixed inner
cone has two openings, left and right, with turning vanes; the outer cone turns one constant-area cutout over them,
driven by the pedals. Angles are about the thruster axis from +Y (right), 90 up, 180 left.

- `fcs/notar/` geometry: `thruster-seam-x-ft` 21.6, `thruster-end-x-ft` 24.5,
  `thruster-axis-z-ft` -3.173, `thruster-radius-front-ft` 0.97, `thruster-radius-aft-ft` 0.75, `cutout-x0-ft` 21.85,
  `cutout-x1-ft` 24.15, `opening-half-angle-deg` alpha 60, `cutout-half-angle-deg` beta 49.29.
- beta = A_n / (L r_m), L 2.30 ft, r_m 0.864 ft. It needs beta <= alpha <= (180 - beta) / 2 = 65.4, so the right
  opening is shut at neutral.
- `sleeve-angle-deg`, piecewise linear in pedal: -1 -> 180, 0 -> 120, +1 -> 0. The mesh's turning cone is drawn at 120,
  and `sleeve-turn-deg`, the angle less that, turns it about the boom axis.
- Exit area = L r_m x the arc where the cutout overlaps an opening. Neutral exposes beta on the left (= A_n). Full left
  and full right both expose 2 beta (3.42 ft2), reached at |pedal| 0.82 and 0.91. Between pedal 0.09 and 0.41 both
  exits are partly open; the net force crosses zero at +0.25.
- The force acts at the cutout's centre, x 23.0 on the axis z -3.173 (`orca.xml`).

## Circulation-control boom

The slot (x 9.29-18.09) is cut into 8 segments, each with its own wake speed, Cmu, Cy and arm (Aero.xml):
the force goes as Cy(Cmu) w^2 with Cmu ~ 1/w^2, so one slot-average wake speed is biased.
A segment's wake speed is vi-fps x shape(r) / c^2 x the Coleman factor, less the boom's own w + q l:
- shape(r): BEMT hover inflow with Prandtl tip loss [L06 ch. 3] for this blade, as the annulus mean F lambda over
  sqrt(CT/2), the uniform inflow FGRotor reports. Disc mean 0.964, peak 1.10 at 0.82 R, 0 at the tip.
- r: the disc radius the segment's wake came from, its station divided by the Landgrebe contraction c at its depth
  [L72]; c = 0.81 at the boom, so the wake edge is x 14.3 ft and segments 6-8 get nothing in hover.
- Eight segments are within 3 % of the converged boom moment (8 to 32 segments: 1600 to 1555 lb ft in hover).
- Slot width h: Cmu 0.4 [Bakir eq. 3.64; Mechanika 2018; US 4,200,252] at the fastest segment crossflow of the
  trimmed DGW hover (49.4 fps), so no segment in the wake sits below the design point. Vj / w is 4.14 there.
- Neutral jet area A_n: the trimmed hover's jet moment / (arm x 2 dp), so the DGW sea-level hover trims at pedal 0;
  h and A_n are iterated to a consistent pair.

## Governor

A feed-forward of the shaft load (rotor torque x speed + fan + gearbox, `fcs/notar/shaft-load-hp`, last step) over the
engine's lapsed maximum, plus the PID as a trim, goes through the inverse engine map. The integrator holds while the
summed demand is pinned. `fcs/throttle-cmd-norm[0]` above 0.5 engages it; below, the throttle is the N1 command.

## Results

Each case is trimmed at dt 1/240 and 1/1000 s; they agree to the printed digit.

| case | model | NASA [J20] |
|---|---|---|
| hover OGE, DGW, SL ISA: total power | 325.8 hp (aircraft FM 0.637) | 326.3 hp (FM 0.636) |
| rotor power at its thrust | 265.5 hp (rotor FM 0.797) | 285.5 hp (FM 0.741) |
| fan + gearbox | 47.8 + 12.5 hp | - |
| collective / pitch / roll / cyclic / pedal | 20.1 deg root / +3.0 / -4.9 / 0, 0 / 0.00 | - |
| anti-torque split, hover | boom 33 %, jet 67 %; slot flow 10.7 % of the fan's | boom 55-70 %; slot < 10 % |
| hover 6000 ft ISA | 347.7 hp (FM 0.648), pedal -0.06 | 354.2 hp |
| 105.4 kt 10000 ft | 225.1 hp (fan 44.5), L/De 5.62, rotor L/De 10.2, fins 70 %, pedal +0.19 | 250 hp, L/De 5.06, rotor 7.45, fins 50-90 % |
| max speed at MCP 364 hp, 10000 ft | > 130 kt (277 hp at 130); 140 kt does not trim | VH 117.85 kt |
| autorotation entry, 60 kt, cut + 1 s delay | 80 % rpm at 2.3 s, 100 % at 6 s | - |
| air start in the DGW hover | within 1 % of nominal rpm from 0.8 s (1/240) / 1.1 s (1/1000) | - |
| ground idle 30 s | no NaN, drift < 0.1 ft | - |

## Known limits

- The boom's share in hover is 33 %, below the 55-70 % J20 quotes for NOTAR in general (MD's design, not a QSMR
  calculation). The contracted wake ends at x 14.3 ft, so the outer 40 % of NASA's slot, with the longest arms, gets
  nothing; the wake is an isolated rotor's (no fuselage or boom blockage, no swirl); Cy comes from one 2D test.
- The slot takes 10.7 % of the fan's flow in the DGW sea-level hover, over J20's "less than 10%": Cmu 0.4 on the
  in-wake segments with NASA's slot length forces it. 10 % would need Cmu 0.37-0.39.
- The rotor is too efficient: FM 0.80 against 0.741 and cruise rotor L/De 10 against 7.45, because FGRotor has no
  induced-power factor, stall or compressibility. Hover power lands within 2 % of NASA's only because the fan draws
  48-55 hp; cruise power is 10 % under NASA's and VH is above 130 kt.
- The boom gives nothing from 40 kt (Dawson-Thompson: 0.2-0.3 of hover). The hover contraction is used at every speed,
  sideward flow on the boom is not modelled, and the model cruises 1.5-3 deg nose-up.
- Rotor energy is low: KE/P 0.92 s against NDARC's 3 s for a good autorotation, so rpm drops to 80 % in an entry.
- A collective pull droops the rotor: +3 deg in hover takes it to 94.6 %, +5 deg to 88.9 % (power-limited). The
  governor has no collective anticipation, and the anti-torque falls with rpm squared.
- Yaw in hover has no damping of its own: the pedal step response is a pure integrator, so a yaw stop overshoots.
- Ground effect is off (`groundeffectexp` 0) until the JSBSim build PteroSim ships applies FGRotor's ground-effect
  factor once per step instead of compounding it; it will then be fitted to Cheeseman-Bennett.
- One engine stands for two: there is no one-engine-inoperative case. The map, lapse and spool are a template.
- An air start begins off nominal rotor speed: JSBSim's set-running steady state steps the transmission at 0.5 s and
  leaves the rotor at 104.5 % (dt 1/240) or 89.2 % (1/1000); the governor brings it within 1 % in 1.2 s.
- The DGW cannot hover OGE at 10000 ft ISA (363 hp needed, 351 hp available).
- The thruster's cone turns only in a PteroSim that turns a `Visual.xml` surface by any JSBSim property; an older one
  shows it standing at neutral.
- Not modelled: fuselage pitching moment, rotor swirl at the fins, induced drag of the surfaces, brakes, nose-wheel
  steering, tail contacts.
- Departures from NASA's numbers: sfc is 0.574 in J20's Table 2 and 0.54 in its text. NASA's VSP slot is 0.077 ft wide,
  28 % of the hover's fan flow; the sized slot is 0.0233 ft. The VSP jet window is 1.28 ft2; the sleeve opens 1.71 ft2
  at neutral and 3.42 ft2 at full pedal.
