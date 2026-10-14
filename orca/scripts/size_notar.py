"""Iterates slot width h and neutral jet area A_n in Systems/notar.xml to a consistent pair:
h for Cmu 0.4 at the fastest slot segment's crossflow, A_n so the DGW hover trims at pedal 0."""
import math
import os
import re
import statistics
import sys

ORCA = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # the aircraft folder
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import validate as v  # noqa: E402

NOTAR = os.path.join(ORCA, 'Systems', 'notar.xml')
CMU_DESIGN = 0.4          # [Bakir eq. 3.64; Mechanika 2018; US 4,200,252]
DT = 1.0 / 240.0
MAX_ITERATIONS = 10


def pattern(name):
    return re.compile(r'(<property value=")([^"]+)(">\s*fcs/notar/' + re.escape(name) + r' )')


def get(text, name):
    m = pattern(name).search(text)
    if not m:
        raise RuntimeError(f'fcs/notar/{name} not in notar.xml')
    return float(m.group(2))


def put(text, name, value):
    new, n = pattern(name).subn(lambda m: m.group(1) + value + m.group(3), text)
    if n != 1:
        raise RuntimeError(f'fcs/notar/{name}: {n} matches')
    return new


def main():
    for it in range(1, MAX_ITERATIONS + 1):
        text = open(NOTAR, encoding='utf-8').read()
        dp = get(text, 'boom-pressure-design-psf')
        x0, x1 = get(text, 'cutout-x0-ft'), get(text, 'cutout-x1-ft')
        seam, end = get(text, 'thruster-seam-x-ft'), get(text, 'thruster-end-x-ft')
        r0, r1 = get(text, 'thruster-radius-front-ft'), get(text, 'thruster-radius-aft-ft')
        radius = lambda x: r0 + (r1 - r0) * (x - seam) / (end - seam)
        area_per_rad = (x1 - x0) * statistics.fmean((radius(x0), radius(x1)))

        a = v.hover_report(DT, v.LOW_ALT_FT, f'iteration {it} (h {get(text, "slot-width-ft")}, '
                                             f'A_n {get(text, "jet-area-neutral-ft2")})')
        segments = range(1, 1 + sum(1 for k in a if k.startswith('slot') and k.endswith('_w')))
        w = max(a[f'slot{k}_w'] for k in segments)
        boom_d = float(re.search(r'<property value="([^"]+)">\s*aero/notar/boom-diameter-ft ',
                                 open(os.path.join(ORCA, 'Aero.xml'), encoding='utf-8').read()).group(1))
        h = CMU_DESIGN * boom_d * a['rho'] * w ** 2 / (4.0 * dp)  # Cmu = 2 (h/D) Vj^2 / w^2, Vj^2 = 2 dp / rho
        arm = (a['jet_x_in'] - a['cg_x_in']) / v.IN_PER_FT
        moment = a['jet_lbs'] * arm
        a_n = moment / (arm * 2.0 * dp)
        beta = math.degrees(a_n / area_per_rad)
        vj = math.sqrt(2.0 * dp / a['rho'])
        print(f'  -> w max {w:.2f} fps, Vj/w {vj / w:.3f}, h {h:.5f} ft; jet moment {moment:.0f} lb ft at {arm:.3f} ft,'
              f' A_n {a_n:.4f} ft2, beta {beta:.3f} deg, pedal {a["pedal"]:+.4f}')
        new = {'slot-width-ft': f'{h:.5f}', 'jet-area-neutral-ft2': f'{a_n:.3f}', 'cutout-half-angle-deg': f'{beta:.2f}'}
        if all(float(val) == get(text, k) for k, val in new.items()):
            print(f'converged: {it - 1} re-sizings, values unchanged at iteration {it}')
            return
        for k, val in new.items():
            text = put(text, k, val)
        open(NOTAR, 'w', encoding='utf-8', newline='').write(text)
    raise RuntimeError(f'no convergence in {MAX_ITERATIONS} iterations')


if __name__ == '__main__':
    main()
