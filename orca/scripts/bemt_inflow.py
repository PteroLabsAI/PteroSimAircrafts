"""Hover inflow over the Orca rotor and its wake at the NOTAR slot; writes the slot segments of Aero.xml.

1. Blade-element momentum theory with Prandtl's tip loss [Leishman, Principles of Helicopter Aerodynamics, 2nd ed.
   2006, ch. 3, BEMT and its tip-loss section], hover (lambda_c = 0), r = y / R:
       lambda(r) = sigma(r) a / (16 F) (sqrt(1 + 32 F theta(r) r / (sigma(r) a)) - 1)
       F = (2 / pi) arccos(exp(-f)),   f = (Nb / 2) (1 - r) / (r phi) = Nb (1 - r) / (2 lambda)   (phi = lambda / r)
       CT = integral from the root cutout to 1 of 4 F lambda^2 r dr
   theta(r) = theta0 + theta_tw r (FGRotor's convention: theta0 at the shaft), theta0 solved so CT matches the
   model's DGW hover. lambda is the inflow at the blade, F lambda its mean round the annulus (the flux the momentum
   balance carries); the boom feels that mean. The table is F lambda / sqrt(CT / 2): the local mean downwash over the
   uniform momentum inflow FGRotor reports as vi-fps (validate.py: vi-fps = sqrt(T / (2 rho A)) in hover).

2. Wake contraction, Landgrebe's generalised hover wake [Landgrebe, JAHS 17(4) 1972; as given in Leishman ch. 10],
   tip vortex at wake age psi (rad), z positive up from the tip-path plane, where the tip vortices leave:
       r_tip / R = A + (1 - A) exp(-Lambda psi),   A = 0.78,   Lambda = 0.145 + 27 CT
       z_tip / R = k1 psi up to the next blade (psi <= 2 pi / Nb), then k1 2 pi / Nb + k2 (psi - 2 pi / Nb)
       k1 = -0.25 (CT / sigma + 0.001 theta_tw),   k2 = -(1.41 + 0.0141 theta_tw) sqrt(CT / 2),   theta_tw in deg
   Streamlines inside contract with the tip vortex (chosen: self-similar), so a station y off the wake axis at depth z
   comes from disc radius y / c(z), c = r_tip / R, and continuity grows its downwash by 1 / c^2.

3. The slot is cut into N equal segments (default 8, chosen); each gets its station in the disc plane (shaft tilt
   3 deg), depth below the tip-path plane (the disc-plane depth + R sin a0, coning a0 FGRotor's in the DGW hover),
   contraction and arm, written into Aero.xml between the markers with the shape and Cy tables.

Run: python scripts/bemt_inflow.py [N]
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

FDM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AERO = os.path.join(FDM, 'Aero.xml')
BEGIN = '  <!-- BEGIN slot segments'
END = '  <!-- END slot segments -->\n'

# Blade, 450 ft/s QSMR [J20 Table 2 and p.7; V22 Table 1]
BLADES = 6
RADIUS_FT = 17.64
FT_PER_M = 1.0 / 0.3048
ROOT_CHORD_FT = 0.290 * FT_PER_M
TIP_CHORD_FT = 0.174 * FT_PER_M          # 60 % taper
TAPER_START = 0.94                       # r where the taper starts
ROOT_CUTOUT = 0.15
TWIST_DEG = -12.0
LIFT_SLOPE = 5.7                         # per rad, as Engines/orca_main_rotor.xml [NT ch.12]
SOLIDITY = 0.0994                        # thrust-weighted [J20 Table 2], Landgrebe's CT / sigma
NOMINAL_RPM = 243.5                      # [V22 Table 3]

# DGW hovers trimmed by validate.py (validate_v3.log): thrust lb, density slug/ft3 (ISA 500 ft and 6000 ft)
HOVERS = (('sea level (500 ft)', 3959.4, 0.0023423), ('6000 ft', 3955.9, 0.0019867))
MODEL_COLLECTIVE_RAD = 0.3504            # FGRotor's theta0 in that sea-level hover (validate_v3.log 20.08 deg)
CONING_RAD = 0.1463                      # FGRotor's a0 there (8.38 deg; 8.47 at 6000 ft): the tips fly R sin a0 above the hub

# Landgrebe 1972, as above
LANDGREBE_A = 0.78
LANDGREBE_LAMBDA = (0.145, 27.0)         # Lambda = 0.145 + 27 CT
LANDGREBE_K1 = (0.25, 0.001)             # k1 = -0.25 (CT / sigma + 0.001 theta_tw)
LANDGREBE_K2 = (1.41, 0.0141)            # k2 = -(1.41 + 0.0141 theta_tw) sqrt(CT / 2)

# Slot and boom, structural frame X aft Z up [VSP NOTAR_slot, tail boom, Rotor_1]
SLOT_X_FT = (9.291, 18.094)
BOOM_AXIS_Z_FT = -3.27
SHAFT_TILT_DEG = 3.0                     # disc front tip down

# Boom side force per length / (q D) vs Cmu [Mechanika 2018 Table 2, Sampatacos' experiment]; 0 at Cmu 0 (no blowing,
# no circulation; linear to the first point assumed); held above 0.5, where the gain saturates [Luo, ICAS 2000]
CY_VS_CMU = ((0.0, 0.0), (0.1, 1.12), (0.2, 4.98), (0.3, 5.68), (0.4, 5.98), (0.5, 6.51))

SHAPE_R = (0.0, ROOT_CUTOUT, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.85, 0.9, 0.94, 0.96, 0.98, 0.99, 0.995, 0.999,
           1.0)                          # chosen: F goes as sqrt(1 - r) at the tip
STATIONS = 4000                          # radial integration stations (chosen: CT to 1e-6)
F_TOL, F_ITER = 1e-12, 200               # Prandtl F fixed point (chosen)
THETA_TOL = 1e-10
SEGMENTS = 8                             # chosen; validate.py shows 8 and 16 agree


def chord_ft(r):
    if r <= TAPER_START:
        return ROOT_CHORD_FT
    return ROOT_CHORD_FT + (TIP_CHORD_FT - ROOT_CHORD_FT) * (r - TAPER_START) / (1.0 - TAPER_START)


def inflow(r, theta0):
    """(lambda, F) at r: the BEMT inflow at the blade and Prandtl's F."""
    if r <= ROOT_CUTOUT or r >= 1.0:
        return 0.0, 0.0
    sa = BLADES * chord_ft(r) / (math.pi * RADIUS_FT) * LIFT_SLOPE
    theta = theta0 + math.radians(TWIST_DEG) * r
    f_loss = 1.0
    for _ in range(F_ITER):
        lam = sa / (16.0 * f_loss) * (math.sqrt(1.0 + 32.0 * f_loss * theta * r / sa) - 1.0)
        new = 2.0 / math.pi * math.acos(math.exp(-BLADES * (1.0 - r) / (2.0 * lam)))
        if abs(new - f_loss) < F_TOL:
            return lam, new
        f_loss = new
    raise RuntimeError(f'Prandtl F did not converge at r {r}')


def thrust_coefficient(theta0):
    dr = (1.0 - ROOT_CUTOUT) / STATIONS
    ct = 0.0
    for i in range(STATIONS):
        r = ROOT_CUTOUT + (i + 0.5) * dr
        lam, f_loss = inflow(r, theta0)
        ct += 4.0 * f_loss * lam * lam * r * dr
    return ct


def solve_theta0(ct):
    lo, hi = -math.radians(TWIST_DEG), math.radians(45.0)   # zero tip pitch .. far past stall
    if not thrust_coefficient(lo) < ct < thrust_coefficient(hi):
        raise RuntimeError(f'CT {ct} outside the bracket')
    while hi - lo > THETA_TOL:
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if thrust_coefficient(mid) < ct else (lo, mid)
    return 0.5 * (lo + hi)


def shape(r, theta0, ct):
    lam, f_loss = inflow(r, theta0)
    return f_loss * lam / math.sqrt(ct / 2.0)


def interpolate(table, x):
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    raise ValueError(x)


def contraction(depth_ft, ct):
    k1 = -LANDGREBE_K1[0] * (ct / SOLIDITY + LANDGREBE_K1[1] * TWIST_DEG)
    k2 = -(LANDGREBE_K2[0] + LANDGREBE_K2[1] * TWIST_DEG) * math.sqrt(ct / 2.0)
    first = 2.0 * math.pi / BLADES
    z = -depth_ft / RADIUS_FT
    psi = z / k1 if z >= k1 * first else first + (z - k1 * first) / k2
    return LANDGREBE_A + (1.0 - LANDGREBE_A) * math.exp(-(LANDGREBE_LAMBDA[0] + LANDGREBE_LAMBDA[1] * ct) * psi)


def xml_value(path, name):
    """A <property value=...> of an XML file, by property name."""
    for p in ET.parse(os.path.join(FDM, path)).getroot().iter('property'):
        if p.text.strip() == name and 'value' in p.attrib:
            return float(p.attrib['value'])
    raise KeyError(f'{name} not in {path}')


def segments(n, ct):
    slot_length = SLOT_X_FT[1] - SLOT_X_FT[0]
    if abs(slot_length - xml_value('Systems/notar.xml', 'fcs/notar/slot-length-ft')) > 1e-3:
        raise RuntimeError('SLOT_X_FT and notar.xml fcs/notar/slot-length-ft disagree')
    aerorp_x = float(ET.parse(os.path.join(FDM, 'Metrics.xml')).getroot().find("location[@name='AERORP']/x").text)
    tilt = math.radians(SHAFT_TILT_DEG)
    out = []
    for k in range(n):
        x = SLOT_X_FT[0] + (k + 0.5) * slot_length / n
        depth = x * math.sin(tilt) - BOOM_AXIS_Z_FT * math.cos(tilt) + RADIUS_FT * math.sin(CONING_RAD)
        out.append(dict(x=x, disc_x=x * math.cos(tilt) + BOOM_AXIS_Z_FT * math.sin(tilt), depth=depth,
                        c=contraction(depth, ct), arm=x - aerorp_x))
    return out


def table_xml(var, rows, indent):
    pad = ' ' * indent
    body = '\n'.join(f'{pad}    {x:<6.3f} {y:.4f}' for x, y in rows)
    return (f'{pad}<table>\n{pad}  <independentVar> {var} </independentVar>\n{pad}  <tableData>\n{body}\n'
            f'{pad}  </tableData>\n{pad}</table>')


def segment_xml(k, n, s, shape_rows):
    p = f'aero/notar/slot{k}'
    return f'''
  <!-- segment {k} of {n}: slot x {s["x"]:.3f} ft -->
  <property value="{s["disc_x"]:.3f}"> {p}/disc-x-ft </property>
  <property value="{s["depth"]:.3f}"> {p}/depth-ft </property>
  <property value="{s["c"]:.4f}"> {p}/contraction </property>
  <property value="{s["arm"]:.3f}"> {p}/arm-aft-ft </property>
  <function name="{p}/origin-aft-ft">
    <quotient>
      <difference>
        <property> {p}/disc-x-ft </property>
        <product><property> {p}/depth-ft </property><property> aero/wake/drift-aft </property></product>
      </difference>
      <property> {p}/contraction </property>
    </quotient>
  </function>
  <function name="{p}/origin-left-ft">
    <quotient>
      <product><property> {p}/depth-ft </property><property> aero/wake/drift-right </property></product>
      <property> {p}/contraction </property>
    </quotient>
  </function>
  <function name="{p}/origin-r-norm">
    <quotient>
      <sqrt>
        <sum>
          <pow><property> {p}/origin-aft-ft </property><value> 2.0 </value></pow>
          <pow><property> {p}/origin-left-ft </property><value> 2.0 </value></pow>
        </sum>
      </sqrt>
      <property> aero/wake/rotor-radius-ft </property>
    </quotient>
  </function>
  <function name="{p}/crossflow-fps">
    <difference>
      <product>
        <property> propulsion/engine[0]/vi-fps </property>
        <property> aero/wake/flowing-down </property>
{table_xml(f"{p}/origin-r-norm", shape_rows, 8)}
        <pow><property> {p}/contraction </property><value> -2.0 </value></pow>
        <sum>
          <value> 1.0 </value>
          <product>
            <property> aero/wake/coleman-gain </property>
            <sum>
              <product><property> {p}/origin-aft-ft </property><property> velocities/u-aero-fps </property></product>
              <product><property> {p}/origin-left-ft </property><property> velocities/v-aero-fps </property></product>
            </sum>
          </product>
        </sum>
      </product>
      <sum>
        <property> velocities/w-aero-fps </property>
        <product><property> velocities/q-rad_sec </property><property> {p}/arm-aft-ft </property></product>
      </sum>
    </difference>
  </function>
  <function name="{p}/cmu">
    <ifthen>
      <gt><property> {p}/crossflow-fps </property><value> 0.0 </value></gt>
      <quotient>
        <property> aero/notar/cmu-w2-fps2 </property>
        <pow><property> {p}/crossflow-fps </property><value> 2.0 </value></pow>
      </quotient>
      <value> 0.0 </value>
    </ifthen>
  </function>
  <function name="{p}/force-lbs">
    <ifthen>
      <gt><property> {p}/crossflow-fps </property><value> 0.0 </value></gt>
      <product>
{table_xml(f"{p}/cmu", CY_VS_CMU, 8)}
        <value> 0.5 </value>
        <property> atmosphere/rho-slugs_ft3 </property>
        <pow><property> {p}/crossflow-fps </property><value> 2.0 </value></pow>
        <property> aero/notar/boom-diameter-ft </property>
        <property> aero/notar/segment-length-ft </property>
      </product>
      <value> 0.0 </value>
    </ifthen>
  </function>
'''


def block_xml(segs, shape_rows, theta0, ct):
    n = len(segs)
    forces = ''.join(f'\n      <property> aero/notar/slot{k}/force-lbs </property>' for k in range(1, n + 1))
    moments = ''.join(f'\n        <product><property> aero/notar/slot{k}/arm-aft-ft </property>'
                      f'<property> aero/notar/slot{k}/force-lbs </property></product>' for k in range(1, n + 1))
    return (f'''{BEGIN}: written by scripts/bemt_inflow.py {n}, do not edit. Derived there: the shape table (BEMT
       with Prandtl tip loss, theta0 {math.degrees(theta0):.2f} deg for CT {ct:.5f}, the DGW sea-level hover), each segment's
       station in the disc plane, depth below the tip-path plane, Landgrebe contraction there and arm aft of the AERORP.
       Cy vs Cmu: Sampatacos [Mechanika 2018 Table 2], held above 0.5 [Luo, ICAS 2000]. -->
  <property value="{n}"> aero/notar/slot-segment-count </property>
''' + ''.join(segment_xml(k, n, s, shape_rows) for k, s in enumerate(segs, 1)) + f'''
  <function name="aero/notar/boom-force-lbs">
    <sum>{forces}
    </sum>
  </function>
  <function name="aero/notar/boom-yaw-lbsft">
    <product>
      <value> -1.0 </value>
      <sum>{moments}
      </sum>
    </product>
  </function>
{END}''')


def write_block(text):
    with open(AERO, encoding='utf-8') as f:
        aero = f.read()
    if aero.count(BEGIN) != 1 or aero.count(END) != 1:
        raise RuntimeError(f'Aero.xml needs exactly one "{BEGIN.strip()}" and one "{END.strip()}"')
    start, end = aero.index(BEGIN), aero.index(END) + len(END)
    with open(AERO, 'w', encoding='utf-8', newline='\n') as f:
        f.write(aero[:start] + text + aero[end:])


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else SEGMENTS
    tip_speed = NOMINAL_RPM * math.pi / 30.0 * RADIUS_FT
    area = math.pi * RADIUS_FT ** 2
    solved = {}
    for label, thrust, rho in HOVERS:
        ct = thrust / (rho * area * tip_speed ** 2)
        theta0 = solve_theta0(ct)
        solved[label] = (theta0, ct)
        disc_mean = sum(shape(ROOT_CUTOUT + (i + 0.5) * (1 - ROOT_CUTOUT) / STATIONS, theta0, ct)
                        * 2.0 * (ROOT_CUTOUT + (i + 0.5) * (1 - ROOT_CUTOUT) / STATIONS) * (1 - ROOT_CUTOUT) / STATIONS
                        for i in range(STATIONS))
        peak = max((shape(r / 1000.0, theta0, ct), r / 1000.0) for r in range(1000))
        print(f'{label}: CT {ct:.5f}, BEMT theta0 {math.degrees(theta0):.2f} deg (model collective '
              f'{math.degrees(MODEL_COLLECTIVE_RAD):.2f} gives CT {thrust_coefficient(MODEL_COLLECTIVE_RAD):.5f}); '
              f'F lambda / sqrt(CT/2): disc mean {disc_mean:.3f}, peak {peak[0]:.3f} at r {peak[1]:.3f}')
        print('  r     ' + ' '.join(f'{r:5.3f}' for r in SHAPE_R))
        print('  shape ' + ' '.join(f'{shape(r, theta0, ct):5.3f}' for r in SHAPE_R))
    theta0, ct = solved[HOVERS[0][0]]
    rows = [(r, shape(r, theta0, ct)) for r in SHAPE_R]
    fine = [i / 2000.0 for i in range(int(ROOT_CUTOUT * 2000) + 1, 2000)]
    err = max(abs(interpolate(rows, r) - shape(r, theta0, ct)) for r in fine if SHAPE_R[2] <= r <= SHAPE_R[-2])
    print(f'table: worst linear-interpolation error {err:.4f} from r {SHAPE_R[2]} to {SHAPE_R[-2]}')
    segs = segments(n, ct)
    ct_high = solved[HOVERS[1][0]][1]
    for k, s in enumerate(segs, 1):
        r_hover = s['disc_x'] / (s['c'] * RADIUS_FT)
        print(f'  segment {k}: x {s["x"]:.3f}, disc x {s["disc_x"]:.3f}, depth {s["depth"]:.3f} ft (z/R '
              f'{s["depth"] / RADIUS_FT:.3f}), contraction {s["c"]:.4f} ({contraction(s["depth"], ct_high):.4f} at '
              f'6000 ft), from disc r {r_hover:.3f}, downwash / vi {interpolate(rows, min(r_hover, 1.0)) / s["c"] ** 2:.3f}')
    write_block(block_xml(segs, rows, theta0, ct))
    print(f'wrote {n} segments into {AERO}')


if __name__ == '__main__':
    main()
