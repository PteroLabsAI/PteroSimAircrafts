"""The unmanned QSMR's paint and relief as one atlas of four projections, each drawn twice: colour, and height for the normal
map (seams cut in, fasteners raised, grilles barred). Blue on top, white below (uav_livery.py), no pinstripes.
Rows: left side, right side (mirrored, lettering unmirrored), roof (looking down), belly (looking up, mirrored so the
right side stays right). Feet, X aft, Y right, Z up. Run: python uav_atlas.py <colour.png> <height.png> <normal.png> <fdm_dir> [features.json]"""
import json
import math
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import orca_notar
import uav_livery as L

X0, X1 = L.ATLAS_X
Z0, Z1 = L.ATLAS_Z
Y0, Y1 = L.ATLAS_Y
SS = 2                        # drawn at twice the atlas's size and filtered down: PIL draws without antialiasing
W, H, PX = L.ATLAS_W * SS, L.ATLAS_H * SS, L.ATLAS_PX * SS
SIDE_H, PLAN_H = L.SIDE_H * SS, L.PLAN_H * SS
ROWS = {k: v * SS for k, v in L.ATLAS_ROWS.items()}
TOP, BOTTOM = L.NAVY, L.WHITE         # blue on top
SEAM_PX, FASTENER_R = 3 * SS, 0.022
FASTENER_STEP, FASTENER_INSET = 0.32, 0.09
SEAM_DEPTH, FASTENER_HEIGHT, BAR_DEPTH = -1.0, 0.6, -0.8     # height-map units: 0 is the skin
NORMAL_STRENGTH = 6.0         # chosen by eye: seams read at a couple of metres without looking embossed
CURVE_STEP = 0.05
FEATURES = []                 # every drawn feature's view and extent, for check_atlas_fit.py
ORCA_EM, PTEROSIM_EM = 0.55, 0.2     # ft, chosen: the name across the boom above the Coanda slot, the maker under it
ORCA_Z, PTEROSIM_Z = -2.87, -3.22     # below the boom's roof-facing top, above the slot
STEP_X, STEP_Z, STEP_W, STEP_H = 1.5, (-6.9, -5.0), 0.45, 0.22   # ft, chosen: kick-in steps up to the handholds
# The Coanda slot as NASA's VSP draws it (Fuselage sub-surface NOTAR_slot): its centreline's ends and its width, ft.
COANDA_SLOT = ((9.291, -3.437), (18.094, -3.366))
COANDA_SLOT_WIDTH = orca_notar.prop(orca_notar.load(sys.argv[4]), 'fcs/notar/slot-width-ft')   # the physics' slot, not VSP's 0.077 marker


class View:
    """One projection: maps the model's feet to the atlas pixel, and draws colour and height side by side."""

    def __init__(self, name, colour, height):
        self.name = name
        self.c = ImageDraw.Draw(colour)
        self.h = ImageDraw.Draw(height)
        self.row0 = ROWS[name]

    def p(self, a, b):
        """(x, z) for the sides, (x, y) for roof and belly."""
        u, w = L.atlas_px(self.name, a, b)
        return u * SS, w * SS

    def box(self, a0, b0, a1, b1):
        (u0, v0), (u1, v1) = self.p(a0, b0), self.p(a1, b1)
        return min(u0, u1), min(v0, v1), max(u0, u1), max(v0, v1)


def hval(v):
    return int(128 + 100 * v)


def paint_at(view, a, b):
    if view.name == 'roof':
        return TOP
    if view.name == 'belly':
        return BOTTOM
    return TOP if b >= L.navy_top(a) else BOTTOM


def ink(rgb):
    return L.WHITE if rgb == L.NAVY else L.NAVY


def seam_colour(rgb):
    return L.SEAM_ON_NAVY if rgb == L.NAVY else L.SEAM_ON_WHITE


def fasteners(v, a0, b0, a1, b1):
    n_a = max(1, int(abs(a1 - a0) / FASTENER_STEP))
    n_b = max(1, int(abs(b1 - b0) / FASTENER_STEP))
    pts = [(a0 + (a1 - a0) * i / n_a, b) for i in range(n_a + 1) for b in (b0, b1)]
    pts += [(a, b0 + (b1 - b0) * j / n_b) for j in range(1, n_b) for a in (a0, a1)]
    r = FASTENER_R * PX
    for a, b in pts:
        u, w = v.p(a, b)
        base = paint_at(v, a, b)
        v.c.ellipse((u - r, w - r, u + r, w + r), fill=tuple(int(c * (0.62 if base == L.NAVY else 0.72)) for c in base))
        v.h.ellipse((u - r, w - r, u + r, w + r), fill=hval(FASTENER_HEIGHT))


def panel(v, a0, b0, a1, b1, r_ft=0.25, with_fasteners=True):
    FEATURES.append((v.name, 'panel', a0, b0, a1, b1))
    base = paint_at(v, (a0 + a1) / 2, (b0 + b1) / 2)
    bx = v.box(a0, b0, a1, b1)
    v.c.rounded_rectangle(bx, radius=r_ft * PX, outline=seam_colour(base), width=SEAM_PX)
    v.h.rounded_rectangle(bx, radius=r_ft * PX, outline=hval(SEAM_DEPTH), width=SEAM_PX)
    if with_fasteners:
        fasteners(v, a0 + FASTENER_INSET, b0 + FASTENER_INSET, a1 - FASTENER_INSET, b1 - FASTENER_INSET)


def seam(v, pts):
    FEATURES.append((v.name, 'seam', min(p[0] for p in pts), min(p[1] for p in pts), max(p[0] for p in pts), max(p[1] for p in pts)))
    for (a0, b0), (a1, b1) in zip(pts, pts[1:]):
        steps = max(1, int(math.hypot(a1 - a0, b1 - b0) / 0.1))
        for k in range(steps):
            t0, t1 = k / steps, (k + 1) / steps
            a, b = a0 + (a1 - a0) * t0, b0 + (b1 - b0) * t0
            seg = [v.p(a, b), v.p(a0 + (a1 - a0) * t1, b0 + (b1 - b0) * t1)]
            v.c.line(seg, fill=seam_colour(paint_at(v, a, b)), width=SEAM_PX)
            v.h.line(seg, fill=hval(SEAM_DEPTH), width=SEAM_PX)


def grille(v, a0, b0, a1, b1, bars_along_a, pitch=0.09, r_ft=0.12, bars=True):
    FEATURES.append((v.name, 'grille', a0, b0, a1, b1))
    """A dark recessed opening, with bars unless they are geometry: the bars stay at skin height, the gaps between them
    are cut in."""
    bx = v.box(a0, b0, a1, b1)
    v.c.rounded_rectangle(bx, radius=r_ft * PX, fill=(14, 16, 20))
    v.h.rounded_rectangle(bx, radius=r_ft * PX, fill=hval(BAR_DEPTH))
    n = int((abs(b1 - b0) if bars_along_a else abs(a1 - a0)) / pitch) if bars else 0
    for i in range(1, n):
        t = i / n
        if bars_along_a:
            seg = [v.p(a0, b0 + (b1 - b0) * t), v.p(a1, b0 + (b1 - b0) * t)]
        else:
            seg = [v.p(a0 + (a1 - a0) * t, b0), v.p(a0 + (a1 - a0) * t, b1)]
        v.c.line(seg, fill=(58, 62, 70), width=max(2, int(0.025 * PX)))
        v.h.line(seg, fill=hval(0.0), width=max(2, int(0.025 * PX)))
    v.c.rounded_rectangle(bx, radius=r_ft * PX, outline=(70, 76, 86), width=SEAM_PX)


def text(v, s, a, b, height_ft):
    half_w = len(s) * height_ft * 0.3       # bold sans: ~0.6 em a character
    FEATURES.append((v.name, 'text ' + s, a - half_w, b - height_ft / 2, a + half_w, b + height_ft / 2))
    font = ImageFont.truetype('arialbd.ttf', max(8, int(height_ft * PX)))
    v.c.text(v.p(a, b), s, fill=ink(paint_at(v, a, b)), font=font, anchor='mm')


def kick_step(v, x, z):
    FEATURES.append((v.name, 'step', x - STEP_W / 2, z - STEP_H / 2, x + STEP_W / 2, z + STEP_H / 2))
    """A recessed foothold: dark inside, cut in, outlined like a seam."""
    base = paint_at(v, x, z)
    bx = v.box(x - STEP_W / 2, z - STEP_H / 2, x + STEP_W / 2, z + STEP_H / 2)
    v.c.rounded_rectangle(bx, radius=0.05 * PX, fill=tuple(int(c * 0.35) for c in base), outline=seam_colour(base), width=SEAM_PX)
    v.h.rounded_rectangle(bx, radius=0.05 * PX, fill=hval(BAR_DEPTH))


def side(v):
    xs = [X0 + i * CURVE_STEP for i in range(int((X1 - X0) / CURVE_STEP) + 1)]
    v.c.rectangle((0, v.row0, W, v.row0 + SIDE_H), fill=TOP)
    v.c.polygon([v.p(x, L.navy_top(x)) for x in xs] + [v.p(x, Z0) for x in reversed(xs)], fill=BOTTOM)
    seam(v, [(-8.2, -8.6), (-8.2, -2.6)])                       # nose cone
    panel(v, -7.6, -6.3, -5.2, -4.2)                             # avionics bay
    panel(v, -4.0, -7.2, 0.8, -3.0, 0.35)                        # payload bay door
    for x in (-3.5, 0.3):                                         # its latches
        v.c.rectangle(v.box(x - 0.14, -7.08, x + 0.14, -6.94), fill=seam_colour(BOTTOM))
        v.h.rectangle(v.box(x - 0.14, -7.08, x + 0.14, -6.94), fill=hval(SEAM_DEPTH))
    u, w = v.p(2.2, -5.0)                                         # fuel filler
    FEATURES.append((v.name, 'fuel filler', 1.98, -5.22, 2.42, -4.78))
    r = 0.22 * PX
    v.c.ellipse((u - r, w - r, u + r, w + r), outline=seam_colour(paint_at(v, 2.2, -5.0)), width=SEAM_PX)
    v.h.ellipse((u - r, w - r, u + r, w + r), outline=hval(SEAM_DEPTH), width=SEAM_PX)
    panel(v, 2.4, -2.55, 4.4, -1.65, 0.15)                        # engine cowling, on the pylon's flat side above the fuselage seam
    grille(v, 2.8, -2.4, 4.0, -1.85, bars_along_a=False)          # engine air intake
    panel(v, 4.5, -2.75, 6.6, -2.05, 0.15)                        # aft cowling
    panel(v, 4.8, -5.2, 6.5, -3.8)                                # engine oil access
    panel(v, -2.5, -2.05, -0.7, -1.62, 0.15)                      # transmission access
    for x in (8.6, 13.0, 18.0):                                   # boom joints
        seam(v, [(x, -4.4), (x, -2.1)])
        fasteners(v, x + 0.15, -4.2, x + 0.15, -2.3)
    for x, z in ((-9.2, -5.9), (-9.2, -6.3)):                     # static ports
        FEATURES.append((v.name, 'static port', x - 0.05, z - 0.05, x + 0.05, z + 0.05))
        u, w = v.p(x, z)
        r = 0.05 * PX
        v.c.ellipse((u - r, w - r, u + r, w + r), fill=seam_colour(paint_at(v, x, z)))
        v.h.ellipse((u - r, w - r, u + r, w + r), fill=hval(SEAM_DEPTH))
    for z in STEP_Z:
        kick_step(v, STEP_X, z)
    if v.name == 'right':                                          # NOTAR Coanda slots: right side only
        (x0, z0), (x1, z1) = COANDA_SLOT
        half = COANDA_SLOT_WIDTH / 2
        outline = [v.p(x0, z0 - half), v.p(x1, z1 - half), v.p(x1, z1 + half), v.p(x0, z0 + half)]
        FEATURES.append((v.name, 'coanda slot', x0, min(z0, z1) - half, x1, max(z0, z1) + half))
        v.c.polygon(outline, fill=(6, 10, 18))
        v.h.polygon(outline, fill=hval(BAR_DEPTH))
    text(v, 'Orca', 15.5, ORCA_Z, ORCA_EM)
    text(v, 'PTEROSIM', 15.5, PTEROSIM_Z, PTEROSIM_EM)
    text(v, 'PS-U1', -6.4, -7.0, 0.5)


def roof(v):
    v.c.rectangle((0, v.row0, W, v.row0 + PLAN_H), fill=TOP)
    panel(v, -2.6, -0.9, -0.4, 0.9, 0.2)                          # transmission deck
    panel(v, 1.0, -1.3, 4.4, 1.3, 0.2)                            # engine top cowling
    # The NOTAR fan inlet behind the mast: its dark recess here, its screen's wires are geometry (build_orca.py).
    grille(v, *L.NOTAR_INLET, bars_along_a=True, r_ft=L.NOTAR_INLET_CORNER, bars=False)
    panel(v, 4.6, -1.0, 7.0, 1.0, 0.2)                            # aft fairing
    panel(v, -6.8, -0.6, -5.0, 0.6, 0.2)                          # nose access
    seam(v, [(-8.2, -2.4), (-8.2, 2.4)])
    for x in (8.6, 13.0, 18.0):
        seam(v, [(x, -1.0), (x, 1.0)])
    for s in (-1, 1):                                             # walkway edges either side of the mast
        seam(v, [(-3.2, s * 1.05), (0.9, s * 1.05)])
    text(v, 'NO STEP', 5.8, 0.0, 0.22)


def belly(v):
    v.c.rectangle((0, v.row0, W, v.row0 + PLAN_H), fill=BOTTOM)
    panel(v, -6.5, -1.2, -3.0, 1.2, 0.3)                          # forward belly access
    panel(v, -2.4, -1.4, 3.6, 1.4, 0.3)                           # fuel bay
    panel(v, 4.2, -0.85, 7.0, 0.85, 0.3)                            # aft equipment bay
    seam(v, [(-8.2, -2.0), (-8.2, 2.0)])
    for x in (8.6, 13.0, 18.0):
        seam(v, [(x, -1.0), (x, 1.0)])
    for x, y in ((1.8, 0.0), (6.8, 0.0)):                        # drains
        FEATURES.append((v.name, 'drain', x - 0.06, y - 0.06, x + 0.06, y + 0.06))
        u, w = v.p(x, y)
        r = 0.06 * PX
        v.c.ellipse((u - r, w - r, u + r, w + r), fill=seam_colour(BOTTOM))
        v.h.ellipse((u - r, w - r, u + r, w + r), fill=hval(SEAM_DEPTH))


colour = Image.new('RGB', (W, H), BOTTOM)
height = Image.new('L', (W, H), hval(0.0))
for name, draw in (('left', side), ('right', side), ('roof', roof), ('belly', belly)):
    draw(View(name, colour, height))
colour = colour.resize((L.ATLAS_W, L.ATLAS_H), Image.LANCZOS)
height = height.resize((L.ATLAS_W, L.ATLAS_H), Image.LANCZOS)
colour.save(sys.argv[1])
if len(sys.argv) > 5:
    json.dump(FEATURES, open(sys.argv[5], 'w'), indent=0)
height.save(sys.argv[2])

# Tangent-space normal map from the height: +u right, +v up the image (OpenGL convention, as glTF and Blender expect).
h = np.asarray(height, dtype=np.float32) / 255.0
dx = np.zeros_like(h)
dy = np.zeros_like(h)
dx[:, 1:-1] = (h[:, 2:] - h[:, :-2]) / 2
dy[1:-1, :] = (h[:-2, :] - h[2:, :]) / 2
n = np.dstack([-dx * NORMAL_STRENGTH, -dy * NORMAL_STRENGTH, np.ones_like(h)])
n /= np.linalg.norm(n, axis=2, keepdims=True)
Image.fromarray(((n * 0.5 + 0.5) * 255).astype(np.uint8)).save(sys.argv[3])
print('atlas', W, H, 'px per ft', round(PX, 1), 'rows', ROWS)
