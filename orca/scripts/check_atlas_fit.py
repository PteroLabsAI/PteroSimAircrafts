"""Checks that every feature drawn on the atlas lands on skin faces that read the view it is drawn on: rays through its
outline and inside hit skin_ref (build_orca.py) and the face's proj_view must match. Feet, X aft, Y right, Z up.
Run: blender -b <orca.blend> -P check_atlas_fit.py -- <features.json>"""
import json
import sys

import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

SAMPLES_A, SAMPLES_B = 9, 5          # chosen: corners, edges and the middle of each feature
VIEW_IDS = {'left': 0, 'right': 1, 'roof': 2, 'belly': 3}
NAMES = {v: k for k, v in VIEW_IDS.items()}
NAMES[-1] = 'tail paint'
RAYS = {'left': ((0, -20, 0), (0, 1, 0)), 'right': ((0, 20, 0), (0, -1, 0)), 'roof': ((0, 0, 20), (0, 0, -1)),
        'belly': ((0, 0, -20), (0, 0, 1))}

features = json.load(open(sys.argv[sys.argv.index('--') + 1]))
ref = bpy.data.objects['skin_ref']
tree = BVHTree.FromObject(ref, bpy.context.evaluated_depsgraph_get())
views = ref.data.attributes['proj_view'].data
bad = 0
for view, kind, a0, b0, a1, b1 in features:
    misses, wrong = 0, {}
    for i in range(SAMPLES_A):
        for j in range(SAMPLES_B):
            a = a0 + (a1 - a0) * i / (SAMPLES_A - 1)
            b = b0 + (b1 - b0) * j / (SAMPLES_B - 1)
            origin, direction = RAYS[view]
            o = Vector(origin) + (Vector((a, 0, b)) if view in ('left', 'right') else Vector((a, b, 0)))
            hit, _, face, _ = tree.ray_cast(o, Vector(direction))
            if hit is None:
                misses += 1
            elif views[face].value != VIEW_IDS[view]:
                got = NAMES[views[face].value]
                wrong[got] = wrong.get(got, 0) + 1
    if (misses or wrong) and kind != 'seam':       # seams are rings: each view draws its arc of them and they overlap
        bad += 1
        print(f'MISFIT {view:5} {kind:22} x {a0:6.2f}..{a1:6.2f}  {"z" if view in ("left", "right") else "y"} {b0:6.2f}..{b1:6.2f}'
              f'  off-skin {misses}  other views {wrong}  of {SAMPLES_A * SAMPLES_B}')
print('features', len(features), 'misfits', bad)
