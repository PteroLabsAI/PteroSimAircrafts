"""The QSMR's (450 ft/s) weight groups from Johnson 2020 Table 2, placed on NASA's VSP geometry: CG and inertia for JSBSim.
Structure is spread over the skin by area, the rotor is a point at the hub (articulated: its blades' inertia lives in the
flapping), engines fill their bays, payload sits in the three seat rows. Feet and lb, X aft, Y right, Z up, hub at origin.
Run: blender -b --factory-startup -P qsmr_mass.py -- <body.obj>"""
import os
import sys

import bpy
from mathutils import Vector

# Johnson 2020 (NTRS 20200000509) Table 2, 450 ft/s column.
DGW, WE, PAYLOAD = 3912.0, 2516.0, 1200.0
STRUCTURE, ROTOR_GROUP, FUSELAGE_GROUP = 1174.0, 330.0, 456.0
PROPULSION, DRIVE = 664.0, 231.0
SYSTEMS = 502.0
ENGINE_LB_PER_HP, ENGINE_HP = 0.50, 2 * 231.0     # p.6: engine weight 0.50 lb/hp
FUEL = DGW - WE - PAYLOAD                          # the design mission's fuel
OTHER_EMPTY = WE - STRUCTURE - PROPULSION - SYSTEMS   # what Table 2's groups leave unnamed; carried with the systems

# Boxes read off the VSP survey (qsmr_survey.vspscript): (min corner, max corner).
TRANSMISSION = (Vector((-0.705, -0.75, -2.955)), Vector((0.871, 0.75, -1.246)))       # Propulsion_1
ENGINE_BAYS = [(Vector((1.23, -2.40, -2.95)), Vector((4.26, 0.0, -1.51))), (Vector((1.23, 0.0, -2.95)), Vector((4.26, 2.40, -1.51)))]
FUEL_TANK = (Vector((4.405, -1.429, -5.095)), Vector((6.310, 0.476, -4.095)))
SEAT_ROWS_X = ((-6.37, -3.41), (-2.68, -0.07), (0.74, 3.35))                         # Payload_033 fpax / mpax / rpax
SEAT_Y, SEAT_Z = (0.22, 2.28), (-8.08, -3.49)                                        # one side; mirrored
CABIN = (Vector((-8.0, -2.6, -8.0)), Vector((3.5, 2.6, -3.0)))                         # systems, avionics, furnishings
BOX_STEPS = 6                                                                         # samples per box edge


def box_points(lo, hi, mass):
    n = BOX_STEPS
    pts = [Vector((lo.x + (hi.x - lo.x) * (i + 0.5) / n, lo.y + (hi.y - lo.y) * (j + 0.5) / n, lo.z + (hi.z - lo.z) * (k + 0.5) / n))
           for i in range(n) for j in range(n) for k in range(n)]
    return [(p, mass / len(pts)) for p in pts]


body_obj = sys.argv[sys.argv.index('--') + 1]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.obj_import(filepath=os.path.abspath(body_obj), forward_axis='Y', up_axis='Z')
skin = bpy.context.selected_objects[0]
me = skin.data
area = sum(f.area for f in me.polygons)
structure_on_skin = STRUCTURE - ROTOR_GROUP
masses = [(Vector(f.center), structure_on_skin * f.area / area) for f in me.polygons]
masses.append((Vector((0, 0, 0)), ROTOR_GROUP))
masses += box_points(*TRANSMISSION, DRIVE)
engines = ENGINE_LB_PER_HP * ENGINE_HP
rest_of_propulsion = PROPULSION - DRIVE - engines   # fuel system, installation, the NOTAR fan and its drive
for bay in ENGINE_BAYS:
    masses += box_points(*bay, (engines + rest_of_propulsion) / len(ENGINE_BAYS))
masses += box_points(*CABIN, SYSTEMS + OTHER_EMPTY)
masses += box_points(*FUEL_TANK, FUEL)
for x0, x1 in SEAT_ROWS_X:
    for s in (-1, 1):
        lo = Vector((x0, min(s * SEAT_Y[0], s * SEAT_Y[1]), SEAT_Z[0]))
        hi = Vector((x1, max(s * SEAT_Y[0], s * SEAT_Y[1]), SEAT_Z[1]))
        masses += box_points(lo, hi, PAYLOAD / (2 * len(SEAT_ROWS_X)))

total = sum(m for _, m in masses)
cg = sum((p * m for p, m in masses), Vector()) / total
ixx = sum(m * ((p.y - cg.y) ** 2 + (p.z - cg.z) ** 2) for p, m in masses)
iyy = sum(m * ((p.x - cg.x) ** 2 + (p.z - cg.z) ** 2) for p, m in masses)
izz = sum(m * ((p.x - cg.x) ** 2 + (p.y - cg.y) ** 2) for p, m in masses)
ixz = sum(m * (p.x - cg.x) * (p.z - cg.z) for p, m in masses)
SLUG = 32.174     # lb per slug
print(f'MASS total {total:.0f} lb (DGW {DGW:.0f}), fuel {FUEL:.0f}, other empty {OTHER_EMPTY:.0f}, skin area {area:.0f} ft2')
print(f'MASS CG x {cg.x:+.3f} y {cg.y:+.3f} z {cg.z:+.3f} ft  (hub at 0,0,0; the CG is {-cg.z:.2f} ft below the hub)')
print(f'MASS inertia slug-ft2: Ixx {ixx / SLUG:.0f}  Iyy {iyy / SLUG:.0f}  Izz {izz / SLUG:.0f}  Ixz {ixz / SLUG:+.0f}')
empty = [(p, m) for p, m in masses]
