"""Offline validation of the Orca FDM against NASA's QSMR, Johnson 2020 (NTRS 20200000509) Table 2, 450 ft/s column.

Flies the model with the jsbsim Python package at dt 1/240 and 1/1000 s: hover out of ground effect, level
flight, the power curve and maximum level speed, an autorotation entry and 30 s of ground idle. A small autopilot
(climb-rate, speed, attitude and heading holds; this script's only) trims each case by flying it to a steady state.
Fails loud: NaN, a saturated rotor brake, a brake limit or engine power copied wrongly from the engine files, a case
that does not settle, or an air start that ends off nominal rotor speed raise an exception.

Run: python scripts/validate.py
"""
import math
import os
import statistics
import xml.etree.ElementTree as ET

import jsbsim

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # the aircraft folder
DTS = (1.0 / 240.0, 1.0 / 1000.0)

# NASA QSMR, 450 ft/s design [J20 Table 2] unless noted
DGW_LB = 3912.0
DISK_AREA_FT2 = math.pi * 17.64 ** 2
ROTOR_FM = 0.741
AIRCRAFT_FM = 0.636
CRUISE_KT = 105.41
AIRCRAFT_LDE = 5.06
ROTOR_LDE = 7.45
VH_KT = 117.85
RATED_HP = 462.0
MRP_PER_MCP = 1.27                 # NDARC typical MRP / MCP ratio [NDARC theory Table 21-3]
CRUISE_ALT_FT = 10000.0            # mission cruise 4000 ft AGL over a 6000 ft ISA take-off [J20 p.5; S21 condition 2]
DESIGN_HOVER_ALT_FT = 6000.0       # hover sizing condition, 6000 ft ISA [S21 condition 1]; CW/sigma 0.10 there [J20 p.8]
BOOM_SHARE_HOVER = (0.55, 0.70)    # [J20 p.6]
FIN_SHARE_CRUISE = (0.50, 0.90)    # [J20 p.6]

KT_TO_FPS = 1.6878099
HP_TO_FTLBS = 550.0
NOMINAL_RPM = 243.5                # Systems/orca_governor.xml
IN_PER_FT = 12.0
HALF_TURN_DEG = 180.0
BETA_TOL_DEG = 0.1                 # chosen: the sleeve cutout's rounding allowance
AREA_TOL_FT2 = 0.01                # chosen: 0.6 % of the neutral jet area
# ERF Fig. 4 with the cutout twice the neutral opening: (pedal, left area / A_n, right area / A_n)
SLEEVE_SCHEDULE = ((-1.0, 2.0, 0.0), (0.0, 1.0, 0.0), (1.0, 0.0, 2.0))

# Autopilot: all chosen for this airframe (rotor control power ~7 rad/s^2 per rad in pitch, ~33 in roll, NOTAR
# ~0.9 rad/s^2 per unit pedal in yaw), checked by the cases settling. It holds zero climb rate (no height loop: an
# extra integrator only slows the trim) and starts every case in hover, then accelerates to the case's speed.
K_CLIMB = 0.004          # rad of collective per ft/s of climb-rate error
KI_CLIMB = 0.002
K_SPEED = 0.012          # rad of attitude per ft/s of speed error
KI_SPEED = 0.0008
ACCEL_FPS2 = 2.0 * KT_TO_FPS   # speed command ramp, 2 kt/s
HOVER_CEILING_FT = 6000.0      # cases above this start in forward flight: at 10000 ft ISA hover OGE needs more than
                               # the lapsed 462 hp (NASA sizes WMTO for hover at 6000 ft [S21 condition 1])
FORWARD_START_KT = 60.0        # chosen: near the bottom of the power bucket
START_CYCLIC_RAD_PER_KT = 0.0017   # chosen: initial forward cyclic so a forward-flight start is not an autogyro
ATTITUDE_LIMIT_RAD = 0.35
K_PITCH, KD_PITCH, KI_PITCH = 0.6, 0.35, 0.3
K_ROLL, KD_ROLL, KI_ROLL = 0.27, 0.08, 0.3
K_YAW, KD_YAW, KI_YAW = 2.5, 2.0, 0.5
CYCLIC_LIMIT_RAD = 0.35
COLLECTIVE_RANGE_RAD = (0.0, 0.5)
HOVER_COLLECTIVE_GUESS_RAD = 0.35    # momentum + blade element at CT/sigma 0.084 gives theta0 ~0.346
GROUND_IDLE_COLLECTIVE_RAD = 0.175   # 10 deg root: just above zero thrust (theta0 = 0.75 |twist| B = 8.8 deg)
AUTOROTATION_COLLECTIVE_RAD = 0.21   # 12 deg root, 3 deg at 0.75 R: a low autorotation pitch (chosen)
AUTOROTATION_KT = 60.0               # entry speed, chosen near Vy (76.65 kt [J20 Table 2]) on the safe side
AUTOROTATION_ALT_FT = 2000.0         # chosen: room for the descent
AIR_START_S = 20.0                   # chosen: twice the slowest recovery seen (zero-integrator governor, ~9 s)
AIR_START_BAND = 0.01                # rotor speed band around nominal, 1 % (chosen)

CURVE_KT = (60.0, 80.0, 100.0, 110.0, 120.0, 130.0, 140.0, 150.0, 160.0)
# (kt north, kt east, Dawson-Thompson boom moment / hover, read off Bakir 2008 Fig. 3.8 forward, Fig. 3.7 right)
BOOM_FADE_CASES = ((0.0, 0.0, '1'), (20.0, 0.0, 'no data'), (40.0, 0.0, 'no data'), (60.0, 0.0, '0.21'),
                   (100.0, 0.0, '0.28'), (-10.0, 0.0, 'no data'), (-20.0, 0.0, 'no data'),
                   (0.0, 10.0, '0.7-0.9'), (0.0, 20.0, '-0.2 to -0.1'))
LOW_ALT_FT = 500.0       # "sea level" cases fly 500 ft over sea-level terrain, well out of ground effect
LOW_SPEED_KT = 60.0
GOVERNOR_ON_CMD = 1.0    # fcs/throttle-cmd-norm[0] above the governor's engage-norm
GROUND_IDLE_CMD = 0.0    # governor off, N1 at idle
SETTLE_S = 90.0
AVERAGE_S = 10.0
STEADY_CLIMB_FPS = 0.5
STEADY_SPEED_FPS = 0.5
STEADY_RPM = 1.0
STEADY_YAW_RATE = 0.01


class Unsettled(RuntimeError):
    """The autopilot could not hold the case steady (out of collective, power or cyclic)."""


def load(dt):
    fdm = jsbsim.FGFDMExec(HERE, None)
    fdm.set_debug_level(0)
    fdm.set_dt(dt)   # fixed at load
    if not fdm.load_model_with_paths('orca', HERE, os.path.join(HERE, 'Engines'), os.path.join(HERE, 'Systems'), False):
        raise RuntimeError('orca did not load')
    return fdm


def check_copied_limits(fdm):
    """The systems' copies of the rotor's brake limit and the engine's maximum power match their files."""
    for engine_file, tag, prop in (('orca_main_rotor.xml', 'maxbrakepower', 'fcs/notar/max-brake-power-hp'),
                                   ('orca_turboshaft.xml', 'maxpower', 'fcs/governor/max-power-hp')):
        in_file = float(ET.parse(os.path.join(HERE, 'Engines', engine_file)).getroot().find(tag).text)
        if abs(in_file - fdm[prop]) > 1e-9:
            raise RuntimeError(f'{tag} {in_file} hp in {engine_file}, {prop} {fdm[prop]}')


def check_thruster(dt):
    """The sleeve valve in notar.xml [ERF p.67-1]: cutout sized from A_n, right opening shut at neutral, the pedal
    schedule's end and neutral areas, and the jet force at the cutout's centre on the thruster axis."""
    fdm, _ = start_in_air(dt, LOW_ALT_FT, 0.0)
    n = lambda name: fdm[f'fcs/notar/{name}']
    x0, x1 = n('cutout-x0-ft'), n('cutout-x1-ft')
    radius = lambda x: n('thruster-radius-front-ft') + (n('thruster-radius-aft-ft') - n('thruster-radius-front-ft')) * (
        x - n('thruster-seam-x-ft')) / (n('thruster-end-x-ft') - n('thruster-seam-x-ft'))
    area_per_rad = (x1 - x0) * statistics.fmean((radius(x0), radius(x1)))
    a_n = n('jet-area-neutral-ft2')
    beta, alpha = math.degrees(a_n / area_per_rad), n('opening-half-angle-deg')
    print(f'  thruster: L r_m {area_per_rad:.4f} ft2/rad, cutout half-angle {beta:.3f} deg (A_n / L r_m; file '
          f'{n("cutout-half-angle-deg")}), openings +-{alpha} deg, sleeve at neutral {n("sleeve-neutral-deg"):.1f} deg')
    if abs(beta - n('cutout-half-angle-deg')) > BETA_TOL_DEG:
        raise RuntimeError(f'cutout-half-angle-deg {n("cutout-half-angle-deg")} but A_n / (L r_m) is {beta:.3f} deg')
    if not beta <= alpha <= (HALF_TURN_DEG - beta) / 2.0:
        raise RuntimeError(f'opening half-angle {alpha} deg is outside [{beta:.2f}, {(HALF_TURN_DEG - beta) / 2.0:.2f}]')
    for pedal, left, right in SLEEVE_SCHEDULE:
        fdm['fcs/notar/pedal-cmd-norm'] = pedal
        fdm.run()
        got = (n('jet-area-left-ft2'), n('jet-area-right-ft2'))
        if abs(got[0] - left * a_n) > AREA_TOL_FT2 or abs(got[1] - right * a_n) > AREA_TOL_FT2:
            raise RuntimeError(f'pedal {pedal}: jet areas {got}, expected {left} and {right} x {a_n} ft2')
    jet_ft = [fdm[f'external_reactions/notar_jet/location-{k}-in'] / IN_PER_FT for k in 'xz']
    if not (math.isclose(jet_ft[0], statistics.fmean((x0, x1))) and math.isclose(jet_ft[1], n('thruster-axis-z-ft'))):
        raise RuntimeError(f'orca.xml jet force at x {jet_ft[0]} z {jet_ft[1]} ft, not the cutout centre on the thruster axis')


def finite_or_raise(fdm, where):
    for p in ('propulsion/engine[0]/rotor-rpm', 'propulsion/engine[0]/thrust-lbs', 'velocities/u-fps',
              'attitude/phi-rad', 'attitude/theta-rad', 'velocities/r-rad_sec', 'aero/notar/boom-force-lbs'):
        if not math.isfinite(fdm[p]):
            raise RuntimeError(f'{p} is {fdm[p]} at t {fdm["simulation/sim-time-sec"]:.2f} s ({where})')
    if fdm['propulsion/engine[0]/brake-ctrl-norm'] >= 1.0:
        raise RuntimeError(f'rotor brake saturated ({fdm["propulsion/engine[0]/brake-ctrl-norm"]:.3f}) ({where})')


def clip(x, lo, hi):
    return max(lo, min(hi, x))


class Autopilot:
    """Holds zero climb rate, ground speeds north and east (ramped to their targets) and heading north."""

    def __init__(self, dt, collective_rad):
        self.dt = dt
        self.speed = self.speed_target = 0.0
        self.side = self.side_target = 0.0
        self.collective_i = collective_rad
        self.pitch_cmd_i = 0.0
        self.roll_cmd_i = 0.0
        self.long_i = self.lat_i = self.pedal_i = 0.0
        self.hold_collective = None   # set to fly a fixed collective (autorotation)

    def step(self, f):
        dt = self.dt
        step = ACCEL_FPS2 * dt
        self.speed += clip(self.speed_target - self.speed, -step, step)
        self.side += clip(self.side_target - self.side, -step, step)
        climb_err = -f['velocities/h-dot-fps']
        if self.hold_collective is None:
            self.collective_i += KI_CLIMB * climb_err * dt
            collective = clip(self.collective_i + K_CLIMB * climb_err, *COLLECTIVE_RANGE_RAD)
        else:
            collective = self.hold_collective

        speed_err = self.speed - f['velocities/v-north-fps']
        self.pitch_cmd_i -= KI_SPEED * speed_err * dt
        pitch_cmd = clip(self.pitch_cmd_i - K_SPEED * speed_err, -ATTITUDE_LIMIT_RAD, ATTITUDE_LIMIT_RAD)
        side_err = self.side - f['velocities/v-east-fps']
        self.roll_cmd_i += KI_SPEED * side_err * dt
        roll_cmd = clip(self.roll_cmd_i + K_SPEED * side_err, -ATTITUDE_LIMIT_RAD, ATTITUDE_LIMIT_RAD)

        pitch_err = f['attitude/theta-rad'] - pitch_cmd          # nose high -> forward cyclic (positive)
        self.long_i += KI_PITCH * pitch_err * dt
        longitudinal = clip(self.long_i + K_PITCH * pitch_err + KD_PITCH * f['velocities/q-rad_sec'],
                            -CYCLIC_LIMIT_RAD, CYCLIC_LIMIT_RAD)
        roll_err = roll_cmd - f['attitude/phi-rad']              # need right roll -> right cyclic (positive)
        self.lat_i += KI_ROLL * roll_err * dt
        lateral = clip(self.lat_i + K_ROLL * roll_err - KD_ROLL * f['velocities/p-rad_sec'],
                       -CYCLIC_LIMIT_RAD, CYCLIC_LIMIT_RAD)
        heading_err = math.remainder(-f['attitude/psi-rad'], 2.0 * math.pi)   # need nose right -> right pedal
        self.pedal_i += KI_YAW * heading_err * dt
        pedal = clip(self.pedal_i + K_YAW * heading_err - KD_YAW * f['velocities/r-rad_sec'], -1.0, 1.0)

        f['propulsion/engine[0]/collective-ctrl-rad'] = collective
        f['propulsion/engine[0]/longitudinal-ctrl-rad'] = longitudinal
        f['propulsion/engine[0]/lateral-ctrl-rad'] = lateral
        f['fcs/notar/pedal-cmd-norm'] = pedal


def start_in_air(dt, alt_ft, start_kt):
    """At alt_ft (ISA, terrain at sea level) and start_kt north, engine running, governor engaged."""
    fdm = load(dt)
    check_copied_limits(fdm)
    fdm['ic/terrain-elevation-ft'] = 0.0
    fdm['ic/h-sl-ft'] = alt_ft
    fdm['ic/psi-true-deg'] = 0.0
    fdm['ic/vn-fps'] = start_kt * KT_TO_FPS
    fdm['propulsion/engine[0]/collective-ctrl-rad'] = HOVER_COLLECTIVE_GUESS_RAD
    fdm['propulsion/engine[0]/longitudinal-ctrl-rad'] = START_CYCLIC_RAD_PER_KT * start_kt
    fdm['fcs/throttle-cmd-norm[0]'] = GOVERNOR_ON_CMD
    fdm.run_ic()
    fdm['propulsion/set-running'] = -1
    pilot = Autopilot(dt, HOVER_COLLECTIVE_GUESS_RAD)
    pilot.speed = start_kt * KT_TO_FPS
    return fdm, pilot


def fly(fdm, pilot, seconds, where):
    for _ in range(int(round(seconds / pilot.dt))):
        pilot.step(fdm)
        fdm.run()
        finite_or_raise(fdm, where)


SAMPLES = {
    'power_hp': 'propulsion/engine[0]/power-hp',
    'rotor_power_hp': 'fcs/notar/rotor-power-hp',
    'fan_power_hp': 'fcs/notar/fan-power-hp',
    'gearbox_loss_hp': 'fcs/notar/gearbox-loss-hp',
    'thrust_lbs': 'propulsion/engine[0]/thrust-lbs',
    'torque_lbsft': 'propulsion/engine[0]/torque-lbsft',
    'rpm': 'propulsion/engine[0]/rotor-rpm',
    'collective_rad': 'propulsion/engine[0]/collective-ctrl-rad',
    'longitudinal_rad': 'propulsion/engine[0]/longitudinal-ctrl-rad',
    'lateral_rad': 'propulsion/engine[0]/lateral-ctrl-rad',
    'pedal': 'fcs/notar/pedal-norm',
    'theta_rad': 'attitude/theta-rad',
    'phi_rad': 'attitude/phi-rad',
    'boom_yaw': 'aero/moment/boom-yaw-lbsft',
    'fin_yaw': 'aero/moment/fin-yaw-lbsft',
    'jet_lbs': 'fcs/notar/jet-force-lbs',
    'jet_x_in': 'external_reactions/notar_jet/location-x-in',
    'sleeve_deg': 'fcs/notar/sleeve-angle-deg',
    'jet_left_ft2': 'fcs/notar/jet-area-left-ft2',
    'jet_right_ft2': 'fcs/notar/jet-area-right-ft2',
    'cg_x_in': 'inertia/cg-x-in',
    'slot_flow': 'fcs/notar/slot-flow-cfs',
    'fan_flow': 'fcs/notar/fan-flow-cfs',
    'download_lbs': 'aero/force/airframe-z-lbs',
    'rho': 'atmosphere/rho-slugs_ft3',
    'weight_lbs': 'inertia/weight-lbs',
    'speed_fps': 'velocities/v-north-fps',
    'climb_fps': 'velocities/h-dot-fps',
    'yaw_rate': 'velocities/r-rad_sec',
    'lapse': 'propulsion/engine[0]/EnginePowerVC',
    'fx_prop': 'forces/fbx-prop-lbs',
    'fz_prop': 'forces/fbz-prop-lbs',
}


def trim(dt, alt_ft, speed_kt, name, side_kt=0.0):
    """Hover at alt_ft, accelerate to speed_kt north and side_kt east, settle; returns the averages over the last
    AVERAGE_S."""
    start_kt = 0.0 if alt_ft <= HOVER_CEILING_FT else min(speed_kt, FORWARD_START_KT)
    fdm, pilot = start_in_air(dt, alt_ft, start_kt)
    samples = SAMPLES | {f'slot{k}_{q}': f'aero/notar/slot{k}/{p}' for k in range(1, int(fdm['aero/notar/slot-segment-count']) + 1)
                         for q, p in (('w', 'crossflow-fps'), ('cmu', 'cmu'), ('lbs', 'force-lbs'))}
    pilot.speed_target = speed_kt * KT_TO_FPS
    pilot.side_target = side_kt * KT_TO_FPS
    ramp_s = max(abs(speed_kt - start_kt), abs(side_kt)) * KT_TO_FPS / ACCEL_FPS2
    fly(fdm, pilot, ramp_s + SETTLE_S - AVERAGE_S, name)
    window = int(round(AVERAGE_S / dt))
    sums = dict.fromkeys(samples, 0.0)
    worst = dict(climb=0.0, speed=0.0, rpm=0.0, yaw=0.0)
    for _ in range(window):
        pilot.step(fdm)
        fdm.run()
        finite_or_raise(fdm, name)
        for k, p in samples.items():
            sums[k] += fdm[p]
        worst['climb'] = max(worst['climb'], abs(fdm['velocities/h-dot-fps']))
        worst['speed'] = max(worst['speed'], abs(fdm['velocities/v-north-fps'] - pilot.speed_target),
                             abs(fdm['velocities/v-east-fps'] - pilot.side_target))
        worst['rpm'] = max(worst['rpm'], abs(fdm['propulsion/engine[0]/rotor-rpm'] - NOMINAL_RPM))
        worst['yaw'] = max(worst['yaw'], abs(fdm['velocities/r-rad_sec']))
    avg = {k: v / window for k, v in sums.items()}
    unsettled = (worst['climb'] > STEADY_CLIMB_FPS or worst['speed'] > STEADY_SPEED_FPS
                 or worst['rpm'] > STEADY_RPM or worst['yaw'] > STEADY_YAW_RATE)
    if unsettled:
        raise Unsettled(f'{name} at dt {dt:.5f} did not settle: {worst}, collective {math.degrees(avg["collective_rad"]):.1f} deg,'
                        f' engine {avg["power_hp"]:.0f} hp')
    return avg


def yaw_split(a):
    """Anti-torque yaw moments (lb ft, nose left positive) of boom, jet and fins about the CG, and their shares."""
    jet = a['jet_lbs'] * (a['jet_x_in'] - a['cg_x_in']) / IN_PER_FT
    boom, fin = -a['boom_yaw'], -a['fin_yaw']
    total = boom + jet + fin
    return dict(boom=boom, jet=jet, fin=fin, total=total,
                boom_share=boom / total, jet_share=jet / total, fin_share=fin / total)


def jet_text(a):
    return (f'sleeve {a["sleeve_deg"]:.1f} deg, jet area L {a["jet_left_ft2"]:.3f} R {a["jet_right_ft2"]:.3f} ft2, '
            f'jet {a["jet_lbs"]:.1f} lb')


def ideal_power_hp(thrust_lbs, rho):
    return thrust_lbs * math.sqrt(thrust_lbs / (2.0 * rho * DISK_AREA_FT2)) / HP_TO_FTLBS


def hover_report(dt, alt_ft, label):
    a = trim(dt, alt_ft, 0.0, f'hover {label}')
    s = yaw_split(a)
    p_ideal_w = ideal_power_hp(a['weight_lbs'], a['rho'])
    p_ideal_t = ideal_power_hp(a['thrust_lbs'], a['rho'])
    print(f'  hover OGE {label}, DGW: engine {a["power_hp"]:.1f} hp = rotor {a["rotor_power_hp"]:.1f} + fan '
          f'{a["fan_power_hp"]:.1f} + gearbox {a["gearbox_loss_hp"]:.1f}')
    print(f'    NASA: rotor {p_ideal_t / ROTOR_FM:.1f} hp (FM {ROTOR_FM} at this thrust), total '
          f'{p_ideal_w / AIRCRAFT_FM:.1f} hp (aircraft FM {AIRCRAFT_FM}); model rotor FM {p_ideal_t / a["rotor_power_hp"]:.3f},'
          f' aircraft FM {p_ideal_w / a["power_hp"]:.3f}')
    print(f'    thrust {a["thrust_lbs"]:.0f} lb (download {a["download_lbs"]:.0f} lb = {a["download_lbs"] / a["weight_lbs"]:.1%}'
          f' of W), collective {math.degrees(a["collective_rad"]):.2f} deg root, cyclic long {math.degrees(a["longitudinal_rad"]):+.2f}'
          f' lat {math.degrees(a["lateral_rad"]):+.2f} deg, pitch {math.degrees(a["theta_rad"]):+.2f} roll '
          f'{math.degrees(a["phi_rad"]):+.2f} deg, pedal {a["pedal"]:+.3f}, {jet_text(a)}')
    print(f'    yaw: rotor torque {a["torque_lbsft"]:.0f} lb ft; boom {s["boom"]:.0f} ({s["boom_share"]:.1%}), jet {s["jet"]:.0f}'
          f' ({s["jet_share"]:.1%}), fins {s["fin"]:.0f} ({s["fin_share"]:.1%}); NASA boom {BOOM_SHARE_HOVER[0]:.0%}-'
          f'{BOOM_SHARE_HOVER[1]:.0%}; slot flow {a["slot_flow"] / a["fan_flow"]:.1%} of the fan\'s')
    segments = sorted(int(k[4:-2]) for k in a if k.startswith('slot') and k.endswith('_w'))
    cmu = lambda k: f'{a[f"slot{k}_cmu"]:.2f}' if a[f'slot{k}_w'] > STEADY_CLIMB_FPS else '-'   # else out of the wake
    print('    slot segments root to end, crossflow fps / Cmu / side force lb: ' + ', '.join(
        f'{a[f"slot{k}_w"]:.1f}/{cmu(k)}/{a[f"slot{k}_lbs"]:.1f}' for k in segments))
    return a


def level_report(dt, alt_ft, speed_kt, label):
    a = trim(dt, alt_ft, speed_kt, f'level {speed_kt} kt {label}')
    s = yaw_split(a)
    v = a['speed_fps']
    # rotor propulsive force along the flight path (body x tilted by pitch) and lift (earth up)
    th = a['theta_rad']
    x_force = a['fx_prop'] * math.cos(th) + a['fz_prop'] * math.sin(th)
    lift = a['fx_prop'] * math.sin(th) - a['fz_prop'] * math.cos(th)
    rotor_pi_po_hp = a['rotor_power_hp'] - x_force * v / HP_TO_FTLBS
    print(f'  level {speed_kt:.2f} kt {label}: engine {a["power_hp"]:.1f} hp = rotor {a["rotor_power_hp"]:.1f} + fan '
          f'{a["fan_power_hp"]:.1f} + gearbox {a["gearbox_loss_hp"]:.1f}; aircraft L/De {a["weight_lbs"] * v / (a["power_hp"] * HP_TO_FTLBS):.2f},'
          f' rotor L/De {lift * v / (rotor_pi_po_hp * HP_TO_FTLBS):.2f}')
    print(f'    collective {math.degrees(a["collective_rad"]):.2f} deg, cyclic long {math.degrees(a["longitudinal_rad"]):+.2f} lat '
          f'{math.degrees(a["lateral_rad"]):+.2f}, pitch {math.degrees(a["theta_rad"]):+.2f} roll {math.degrees(a["phi_rad"]):+.2f},'
          f' pedal {a["pedal"]:+.3f}, {jet_text(a)}; yaw: boom {s["boom_share"]:.1%} jet {s["jet_share"]:.1%} fins {s["fin_share"]:.1%}'
          f' of {s["total"]:.0f} lb ft')
    return a


def max_speed(curve, power_key, available):
    """Speed where the power-required curve crosses the available power (linear between trimmed points)."""
    for (v0, a0), (v1, a1) in zip(curve, curve[1:]):
        m0, m1 = available(a0) - a0[power_key], available(a1) - a1[power_key]
        if m0 >= 0.0 > m1:
            return v0 + (v1 - v0) * m0 / (m0 - m1)
    return None


def boom_fade(dt, hover):
    """Boom yaw moment over its hover value in forward, rearward and right sideward flight, against Dawson and
    Thompson's MD flight data as Bakir (2008) Figs. 3.7-3.8 plot them."""
    print('  boom moment / hover boom moment (model; Dawson-Thompson read off Bakir Figs. 3.7-3.8):')
    for north, east, nasa in BOOM_FADE_CASES:
        a = hover if north == east == 0.0 else trim(dt, LOW_ALT_FT, north, f'boom fade {north} N {east} E', east)
        s = yaw_split(a)
        print(f'    {north:+6.1f} kt north {east:+5.1f} kt east: {a["boom_yaw"] / hover["boom_yaw"]:5.2f} (D-T {nasa}),'
              f' boom {s["boom_share"]:.0%} jet {s["jet_share"]:.0%} fins {s["fin_share"]:.0%}, pedal {a["pedal"]:+.2f}, {jet_text(a)}')


def autorotation_entry(dt):
    """Engine cut in level flight at 60 kt; collective held 1 s, then lowered; rotor speed over 6 s."""
    fdm, pilot = start_in_air(dt, AUTOROTATION_ALT_FT, 0.0)
    pilot.speed_target = AUTOROTATION_KT * KT_TO_FPS
    fly(fdm, pilot, pilot.speed_target / ACCEL_FPS2 + SETTLE_S, 'autorotation lead-in')
    fdm['propulsion/cutoff_cmd'] = 1
    t0 = fdm['simulation/sim-time-sec']
    trace = []
    while fdm['simulation/sim-time-sec'] - t0 < 6.0:
        t = fdm['simulation/sim-time-sec'] - t0
        if t >= 1.0:
            pilot.hold_collective = AUTOROTATION_COLLECTIVE_RAD
        pilot.step(fdm)
        fdm.run()
        finite_or_raise(fdm, 'autorotation')
        trace.append((t, fdm['propulsion/engine[0]/rotor-rpm'], fdm['velocities/h-dot-fps'],
                      fdm['propulsion/engine[0]/free-wheel-transmission'], fdm['propulsion/engine[0]/power-hp']))
    marks = [trace[int(round(s / dt))] for s in (0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0)] + [trace[-1]]
    print(f'  autorotation entry, {AUTOROTATION_KT:.0f} kt, {AUTOROTATION_ALT_FT:.0f} ft, DGW: t s / rotor rpm / climb fps / freewheel engaged / engine hp')
    for t, rpm, climb, fw, hp in marks:
        print(f'    {t:4.1f}  {rpm:6.1f}  {climb:+7.1f}  {fw:.0f}  {hp:6.1f}')
    low = min(trace, key=lambda x: x[1])
    print(f'    lowest {low[1]:.1f} rpm ({low[1] / NOMINAL_RPM:.0%}) at {low[0]:.2f} s')


def air_start(dt):
    """Engine set running and the governor engaged at t 0 in a DGW hover, as a simulation starting in the air: rotor
    speed over AIR_START_S. Fails if it ends outside the band."""
    fdm, pilot = start_in_air(dt, LOW_ALT_FT, 0.0)
    trace = []
    for _ in range(int(round(AIR_START_S / dt))):
        pilot.step(fdm)
        fdm.run()
        finite_or_raise(fdm, 'air start')
        trace.append((fdm['simulation/sim-time-sec'], fdm['propulsion/engine[0]/rotor-rpm'] / NOMINAL_RPM))
    low, high = min(trace, key=lambda s: s[1]), max(trace, key=lambda s: s[1])
    outside = [t for t, n in trace if abs(n - 1.0) > AIR_START_BAND]
    if abs(trace[-1][1] - 1.0) > AIR_START_BAND:
        raise RuntimeError(f'air start: rotor at {trace[-1][1]:.1%} after {AIR_START_S:.0f} s')
    print(f'  air start, hover {LOW_ALT_FT:.0f} ft, DGW: rotor from {trace[0][1]:.1%}, lowest {low[1]:.1%} at {low[0]:.2f} s,'
          f' highest {high[1]:.1%} at {high[0]:.2f} s, within {AIR_START_BAND:.0%} from {outside[-1] if outside else 0.0:.2f} s on')


def ground_idle(dt, throttle_cmd, label):
    fdm = load(dt)
    check_copied_limits(fdm)
    fdm['ic/terrain-elevation-ft'] = 0.0
    fdm['ic/h-agl-ft'] = 5.6     # CG 5.695 ft over the tyres' contact points: a little load from the start
    fdm['ic/psi-true-deg'] = 0.0
    fdm['propulsion/engine[0]/collective-ctrl-rad'] = GROUND_IDLE_COLLECTIVE_RAD
    fdm['fcs/throttle-cmd-norm[0]'] = throttle_cmd
    fdm.run_ic()
    fdm['propulsion/set-running'] = -1
    x0, y0 = fdm['position/distance-from-start-lat-mt'], fdm['position/distance-from-start-lon-mt']
    worst_rate = 0.0
    for _ in range(int(round(30.0 / dt))):
        fdm.run()
        finite_or_raise(fdm, 'ground idle')
        worst_rate = max(worst_rate, abs(fdm['velocities/r-rad_sec']))
    drift_ft = math.hypot(fdm['position/distance-from-start-lat-mt'] - x0,
                          fdm['position/distance-from-start-lon-mt'] - y0) / 0.3048
    print(f'  ground, {label}, 30 s: rotor {fdm["propulsion/engine[0]/rotor-rpm"]:.1f} rpm, thrust '
          f'{fdm["propulsion/engine[0]/thrust-lbs"]:.0f} lb, engine {fdm["propulsion/engine[0]/power-hp"]:.1f} hp, '
          f'drift {drift_ft:.2f} ft, heading {fdm["attitude/psi-deg"]:.2f} deg, worst yaw rate {worst_rate:.4f} rad/s, '
          f'CG height {fdm["position/h-agl-ft"]:.2f} ft')


def main():
    for dt in DTS:
        print(f'=== dt 1/{round(1.0 / dt)} s ===')
        check_thruster(dt)
        hover = hover_report(dt, LOW_ALT_FT, f'sea level ISA ({LOW_ALT_FT:.0f} ft AGL)')
        hover_report(dt, DESIGN_HOVER_ALT_FT, '6000 ft ISA')
        level_report(dt, LOW_ALT_FT, LOW_SPEED_KT, 'sea level')
        level_report(dt, LOW_ALT_FT, CRUISE_KT, 'sea level')
        for alt, label in ((LOW_ALT_FT, 'sea level'), (CRUISE_ALT_FT, '10000 ft')):
            curve = []
            for kt in CURVE_KT:
                try:
                    curve.append((kt, trim(dt, alt, kt, f'curve {kt} kt {label}')))
                except Unsettled as e:
                    print(f'    {e}: the curve ends here')
                    break
            vmax = max_speed(curve, 'power_hp', lambda a: RATED_HP * a['lapse'])
            vh = max_speed(curve, 'power_hp', lambda a: RATED_HP / MRP_PER_MCP * a['lapse'])
            print(f'  power curve {label} (kt: engine hp): ' + ', '.join(f'{kt:.0f}: {a["power_hp"]:.0f}' for kt, a in curve))
            last = f'above {curve[-1][0]:.0f} kt (the last trimmed point)'
            print(f'    max level speed at {RATED_HP:.0f} hp (lapsed): {f"{vmax:.1f} kt" if vmax else last}; at MCP '
                  f'{RATED_HP / MRP_PER_MCP:.0f} hp (lapsed): {f"{vh:.1f} kt" if vh else last}; NASA VH {VH_KT} kt')
        level_report(dt, CRUISE_ALT_FT, CRUISE_KT, '10000 ft (NASA cruise)')
        print(f'    NASA cruise {CRUISE_KT} kt: aircraft L/De {AIRCRAFT_LDE} (power {DGW_LB * CRUISE_KT * KT_TO_FPS / AIRCRAFT_LDE / HP_TO_FTLBS:.0f} hp at DGW),'
              f' rotor L/De {ROTOR_LDE}; fins {FIN_SHARE_CRUISE[0]:.0%}-{FIN_SHARE_CRUISE[1]:.0%} of the anti-torque')
        boom_fade(dt, hover)
        autorotation_entry(dt)
        air_start(dt)
        ground_idle(dt, GOVERNOR_ON_CMD, 'flat pitch, governor on')
        ground_idle(dt, GROUND_IDLE_CMD, 'engine at ground idle N1, governor off')


if __name__ == '__main__':
    main()
