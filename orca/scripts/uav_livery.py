"""The PteroSim livery for the unmanned QSMR, shared by the atlas (uav_atlas.py) and the build (build_orca.py).
Colours are sampled from Content/Splash/Splash.png: its navy ground, its steel and the red of the eye. Feet, X aft, Z up."""
import math

WHITE = (238, 241, 245)
NAVY = (10, 32, 62)          # the splash's #02172F lifted a little, so a lit panel still reads blue, not black
STEEL = (61, 87, 106)        # the splash's #3D576A
RED = (205, 93, 82)          # the eye
SEAM_ON_WHITE, SEAM_ON_NAVY = (120, 132, 146), (4, 16, 34)

BELLY_Z = -5.6               # the navy's top along the cabin, as the old two-tone split
SWEEP_FROM_X, SWEEP_TO_X = -1.0, 8.6     # where it rises toward the boom
BOOM_TOP_AT_ROOT, BOOM_TOP_AT_TAIL = -3.3, -3.95   # on the boom it tapers back toward the tail
TAIL_X = 22.0
PIN_GAP, STEEL_W, RED_W = 0.10, 0.10, 0.04           # ft: the pinstripes riding above the navy
# The NOTAR fan inlet on the roof behind the mast, x0, y0, x1, y1 and its corners' radius, chosen by eye on NASA's VSP top:
# the atlas paints its dark recess, build_orca.py lays its screen's wires over that.
NOTAR_INLET, NOTAR_INLET_CORNER = (2.75, -1.25, 4.6, 1.25), 0.12


# The atlas uav_atlas.py draws and build_orca.py maps faces onto: four projections stacked, one scale throughout.
ATLAS_X, ATLAS_Z, ATLAS_Y = (-11.0, 25.0), (-10.0, 1.0), (-5.0, 5.0)
ATLAS_W = 4096
ATLAS_PX = ATLAS_W / (ATLAS_X[1] - ATLAS_X[0])
SIDE_H = round((ATLAS_Z[1] - ATLAS_Z[0]) * ATLAS_PX)
PLAN_H = round((ATLAS_Y[1] - ATLAS_Y[0]) * ATLAS_PX)
ATLAS_ROWS = {'left': 0, 'right': SIDE_H, 'roof': 2 * SIDE_H, 'belly': 2 * SIDE_H + PLAN_H}
ATLAS_H = 2 * SIDE_H + 2 * PLAN_H


def atlas_px(view, a, b):
    """(x, z) on a side view, (x, y) on roof and belly, to the atlas pixel; right and belly are mirrored to read from outside.
    Seen from above with the tail to the right, +Y is up the image; seen from below with the tail to the left, too."""
    u = (a - ATLAS_X[0]) / (ATLAS_X[1] - ATLAS_X[0])
    if view in ('right', 'belly'):
        u = 1.0 - u
    if view in ('left', 'right'):
        v = (ATLAS_Z[1] - b) / (ATLAS_Z[1] - ATLAS_Z[0]) * SIDE_H
    else:
        v = (ATLAS_Y[1] - b) / (ATLAS_Y[1] - ATLAS_Y[0]) * PLAN_H
    return u * ATLAS_W, ATLAS_ROWS[view] + v


def navy_top(x):
    """Height of the navy's upper edge at x."""
    if x <= SWEEP_FROM_X:
        return BELLY_Z
    if x <= SWEEP_TO_X:
        t = (x - SWEEP_FROM_X) / (SWEEP_TO_X - SWEEP_FROM_X)
        s = 0.5 - 0.5 * math.cos(math.pi * t)          # eased, so the sweep starts and ends level
        return BELLY_Z + (BOOM_TOP_AT_ROOT - BELLY_Z) * s
    t = min(1.0, (x - SWEEP_TO_X) / (TAIL_X - SWEEP_TO_X))
    return BOOM_TOP_AT_ROOT + (BOOM_TOP_AT_TAIL - BOOM_TOP_AT_ROOT) * t
