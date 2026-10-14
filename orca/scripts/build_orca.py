"""Blender: the Orca, the unmanned QSMR in PteroSim's livery. The skin is projected onto the four-view atlas (uav_atlas.py),
fittings are modelled on it, then everything is unwrapped once and baked in Cycles to one material: base colour, ORM
(occlusion, roughness, metallic) and a tangent normal map. The rotor is rebuilt from one levelled blade, as before.
Model units are the export's feet (X aft, Y right, Z up); scaled to metres before export.
Run: blender -b --factory-startup -P build_orca.py -- <split_dir> <atlas_dir> <out_dir> <fdm_dir>"""
import math
import os
import sys

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

split_dir, atlas_dir, out_dir, fdm_dir = map(os.path.abspath, sys.argv[sys.argv.index('--') + 1:][:4])
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import orca_notar  # noqa: E402
import uav_livery as L  # noqa: E402
FOOT = 0.3048  # m per international foot; OpenVSP saved this model in feet

# Read off the export (body_parts.py and the rotor measurements).
BOOM_AXIS_Z, BOOM_R = -3.27, 1.0
HUB_R, BLADE_ROOT_R = 1.52, 2.64
BLADE_ROOT_Z = 0.083                 # every blade's root height
SHAFT_TILT_DEG = 3.0                 # VSP Rotor_1 Y_Rel_Rotation 87 deg: the shaft leans forward; blades levelled to it here
GROUND_Z = -9.88
FRONT_WHEEL, MAIN_WHEEL = Vector((-6.695, 0.0, -9.25)), Vector((4.50, 3.03, -9.25))
WHEEL_R, FRONT_W, MAIN_W = 0.63, 0.40, 0.48
GEAR_ZONES = {'front': ((-7.5, -5.4), (-0.35, 0.35), (-10.0, -7.4)), 'left': ((2.7, 5.3), (-3.4, -1.9), (-10.0, -6.6)),
              'right': ((2.7, 5.3), (1.9, 3.4), (-10.0, -6.6))}
EXHAUST_FROM, EXHAUST_TO = Vector((6.85, -1.94, -2.30)), Vector((7.6, -2.10, -2.22))   # from the oval's centre, aft and a little out
EXHAUST_R = 0.45                     # chosen to fill the oval (1.4 x 1.15 ft)
TAIL_FROM_X = 18.5                   # the stabiliser and endplate fins start aft of this (they begin at 18.73)
STAB_TIP_BAND = 0.5                  # ft of the stabiliser's span, at each tip, moved out to meet the fins; covers its rounded cap
BLADE_SHARP_DEG = 30.0              # chosen: a fold sharper than this is an edge, not curvature
# The direct-jet thruster is a sleeve valve (ERF 1983 p.67-1; Bakir 2008 s.2): a fixed inner cone with a left and a right
# opening and turning vanes in them, inside an outer cone the pedals turn, one cutout in it. Its sizes are the NOTAR
# system's (notar.xml); these are only how it is drawn.
THRUSTER_GAP = 0.03                  # ft between the two cones, chosen
THRUSTER_SLEEVE_T = 0.02             # ft, the outer cone's wall, chosen
THRUSTER_VANE_DEPTH, THRUSTER_VANE_PITCH, THRUSTER_VANE_T = 0.15, 0.25, 0.025   # ft, chosen
THRUSTER_SEGMENTS, THRUSTER_STATIONS = 96, 12   # round the axis and along it, chosen: smooth at a metre
LEADING_EDGE_FACING = 0.5            # a tail-surface face turned more than ~60 deg forward takes the erosion strip; chosen
# Fittings set on the skin, chosen by eye (ft): the mechanics' handholds under the cowling, tie-down rings, the VHF whip.
HANDHOLD_X, HANDHOLD_Z, HANDHOLD_STANDOFF = (0.9, 1.7), -3.35, 0.18      # standoff ~5.5 cm: room for a gloved hand
TIE_DOWNS = ((-5.5, 1.1), (6.6, 1.0))
WHIP_X, WHIP_LEN, WHIP_RAKE_DEG = 5.8, 1.4, 45.0
HUB_BOLTS, HUB_BOLT_R = 12, 0.95
STRUT_SOCKET = 0.3                   # ft each oleo runs on past the skin, so its cut end hides inside
STRUT_R = 0.17                       # ft, the oleo's outer cylinder, chosen
STRUT_FAIRING_R, STRUT_FAIRING_L = 0.32, 0.45   # ft, the cone fairing where the strut leaves the skin: base, length, chosen
STRUT_FAIRING_SHARE = 0.5            # of the strut out of the skin the fairing may cover, chosen: the nose has 0.15 ft
STRUT_FAIRING_SUNK = 0.02            # ft the fairing's wide edge tucks into the skin, chosen
STRUT_FAIRING_RAY = 1.0              # ft outside the skin each ray finding its edge starts, chosen: past any nearby skin
STRUT_FAIRING_VERTS = 32             # round the strut, chosen
STRUT_FAIRING_DROP = 0.04            # ft the narrow end reaches past the lowest skin under the fairing, chosen
STRUT_FAIRING_FLARE = 0.4            # ft of fairing radius past the strut per ft of strut out of the skin, chosen: a short
                                     # strut gets a collar, not a dish
DAMPER_SPREAD_DEG = 22.0             # the lead-lag damper's hub end, behind its blade

BAKE_PX = 4096
BAKE_MARGIN_PX = 16                  # chosen: covers mip levels down to 256 px without island bleed
UNWRAP_ANGLE_DEG = 66.0              # Blender's Smart UV default
UNWRAP_MARGIN = 0.002                # of the UV square: ~8 px at 4096
AO_DISTANCE_FT = 3.0                 # chosen: contact shadow round fittings and the cowlings, not the whole belly
AO_SAMPLES, FLAT_SAMPLES = 128, 4    # chosen: AO noise-free at 4096; colour and normal need only the pixel's antialias



NOTAR = orca_notar.load(fdm_dir)
T_SEAM, T_END = orca_notar.prop(NOTAR, 'fcs/notar/thruster-seam-x-ft'), orca_notar.prop(NOTAR, 'fcs/notar/thruster-end-x-ft')
T_AXIS_Z = orca_notar.prop(NOTAR, 'fcs/notar/thruster-axis-z-ft')
T_R0, T_R1 = orca_notar.prop(NOTAR, 'fcs/notar/thruster-radius-front-ft'), orca_notar.prop(NOTAR, 'fcs/notar/thruster-radius-aft-ft')
T_CUT = (orca_notar.prop(NOTAR, 'fcs/notar/cutout-x0-ft'), orca_notar.prop(NOTAR, 'fcs/notar/cutout-x1-ft'))
T_BETA = orca_notar.prop(NOTAR, 'fcs/notar/cutout-half-angle-deg')
T_ALPHA = orca_notar.prop(NOTAR, 'fcs/notar/opening-half-angle-deg')
T_LEFT = orca_notar.prop(NOTAR, 'fcs/notar/left-opening-centre-deg')
T_RIGHT = orca_notar.prop(NOTAR, 'fcs/notar/right-opening-centre-deg')
T_NEUTRAL_DEG = T_LEFT - T_ALPHA      # the sleeve at neutral pedal, as notar.xml schedules it: half the cutout over the left opening

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene


def srgb(r, g, b):
    return tuple(((c / 255.0 + 0.055) / 1.055) ** 2.4 for c in (r, g, b))


def material(name, rgb, rough=0.45, metal=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    p = m.node_tree.nodes['Principled BSDF']
    p.inputs['Base Color'].default_value = (*rgb, 1.0)
    p.inputs['Roughness'].default_value = rough
    p.inputs['Metallic'].default_value = metal
    m.diffuse_color = (*rgb, 1.0)
    return m


def image_node(nodes, path, non_colour, uv_map=None):
    t = nodes.new('ShaderNodeTexImage')
    t.image = bpy.data.images.load(path)
    if non_colour:
        t.image.colorspace_settings.name = 'Non-Color'
    if uv_map:
        uv = nodes.new('ShaderNodeUVMap')
        uv.uv_map = uv_map
        t.id_data.links.new(uv.outputs['UV'], t.inputs['Vector'])
    return t


def atlas_material():
    """The skin: the atlas's colour and relief, read through the 'proj' UV."""
    m = material('skin', (1, 1, 1), rough=0.45)
    nt = m.node_tree
    p = nt.nodes['Principled BSDF']
    col = image_node(nt.nodes, os.path.join(atlas_dir, 'atlas_colour.png'), False, 'proj')
    nt.links.new(col.outputs['Color'], p.inputs['Base Color'])
    nrm = image_node(nt.nodes, os.path.join(atlas_dir, 'atlas_normal.png'), True, 'proj')
    nm = nt.nodes.new('ShaderNodeNormalMap')
    nm.uv_map = 'proj'
    nt.links.new(nrm.outputs['Color'], nm.inputs['Color'])
    nt.links.new(nm.outputs['Normal'], p.inputs['Normal'])
    return m


SKIN = atlas_material()
ACCENT = material('navy', srgb(*L.NAVY), rough=0.4)
TIP = material('tip', srgb(*L.WHITE), rough=0.4)
DARK = material('dark', srgb(46, 48, 52), rough=0.6)
GRILLE = material('grille', srgb(24, 25, 27), rough=0.8)
METAL = material('metal', srgb(120, 122, 126), rough=0.3, metal=0.8)
HOT = material('exhaust', srgb(70, 60, 52), rough=0.5, metal=0.6)
BLADE = material('blade', srgb(26, 32, 42), rough=0.5)
RUBBER = material('rubber', srgb(22, 22, 24), rough=0.9)
CHROME = material('chrome', srgb(205, 207, 210), rough=0.12, metal=1.0)   # the oleo pistons only
STEEL = material('steel', srgb(150, 152, 156), rough=0.45, metal=1.0)      # bolts, links, axles, rings: plated steel, chosen
STRUT = material('strut', srgb(110, 114, 118), rough=0.5, metal=0.3)


def load(name):
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=os.path.join(split_dir, f'{name}.obj'), forward_axis='Y', up_axis='Z')
    ob = (set(bpy.data.objects) - before).pop()
    ob.name = name
    return ob


def inside(c, box):
    return all(lo <= v <= hi for v, (lo, hi) in zip(c, box))


# ---------------------------------------------------------------- body: the skin onto the atlas
body = load('body')

# The export draws its gear as smooth pods: they go, and the struts' upper ends say where the new gear hangs from.
bpy.ops.object.select_all(action='DESELECT')
body.select_set(True)
bpy.context.view_layer.objects.active = body
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.separate(type='LOOSE')
bpy.ops.object.mode_set(mode='OBJECT')
strut_tops, kept = {}, []
for ob in [o for o in bpy.data.objects if o.type == 'MESH']:
    pts = [Vector(v.co) for v in ob.data.vertices]
    c = sum(pts, Vector()) / len(pts)
    zone = next((k for k, b in GEAR_ZONES.items() if inside(c, b)), None)
    if zone is None:
        kept.append(ob)
        continue
    top_z = max(p.z for p in pts)
    if top_z > -8.0:      # a strut; the wheels end at -8.62
        tops = [p for p in pts if p.z > top_z - 0.2]
        strut_tops[zone] = sum(tops, Vector()) / len(tops)
    bpy.data.objects.remove(ob)
# The export leaves the stabiliser's tips short of the endplate fins (a 0.055 ft gap): its tip band is moved out to the
# fins' mid-plane, so the rounded tip sinks into them.
tail = [ob for ob in kept if min(v.co.x for v in ob.data.vertices) > TAIL_FROM_X]
stab = [ob for ob in tail if min(v.co.y for v in ob.data.vertices) < 0 < max(v.co.y for v in ob.data.vertices)]
fins = [ob for ob in tail if ob not in stab]
assert len(stab) == 1 and len(fins) == 2, f'tail pieces: {len(stab)} stabiliser, {len(fins)} fins'
depsgraph = bpy.context.evaluated_depsgraph_get()
fin_trees = [BVHTree.FromObject(ob, depsgraph) for ob in fins]
stab_tip = max(abs(v.co.y) for v in stab[0].data.vertices)
reach = []
for v in stab[0].data.vertices:
    if abs(v.co.y) <= stab_tip - STAB_TIP_BAND:
        continue
    s = math.copysign(1.0, v.co.y)
    ray = Vector((0, s, 0))
    start = Vector((v.co.x, s * (stab_tip - STAB_TIP_BAND), v.co.z))
    near_face = far_face = None
    for tree in fin_trees:
        hit = tree.ray_cast(start, ray)[0]
        if hit is not None:
            near_face = hit
            far_face = tree.ray_cast(hit + ray * 1e-4, ray)[0]
            break
    if near_face is None or far_face is None:
        raise RuntimeError(f'stabiliser vertex {tuple(v.co)} does not see through a fin')
    shift = abs((near_face.y + far_face.y) / 2) - stab_tip     # the fin's mid-surface here, which is thin at its edges
    v.co.y += s * shift
    reach.append(shift)
reach = sum(reach) / len(reach)
print('stabiliser tips moved out', round(reach, 3), 'ft on average')
bpy.ops.object.select_all(action='DESELECT')
for ob in kept:
    ob.select_set(True)
bpy.context.view_layer.objects.active = kept[0]
bpy.ops.object.join()
body = bpy.context.active_object
body.name = 'body'
me = body.data
for f in me.polygons:
    f.use_smooth = True
uncut = body.copy()                 # its normals are put back after the cut: the cut's slivers would shade as facets
uncut.data = me.copy()
scene.collection.objects.link(uncut)


def near_boom_end(f):
    c = f.calc_center_median()
    return c.x > T_SEAM - 0.5 and abs(c.y) < 1.5 and abs(c.z - T_AXIS_Z) < 1.5


bm = bmesh.new()
bm.from_mesh(me)
end = [f for f in bm.faces if near_boom_end(f)]
geom = list({v for f in end for v in f.verts}) + list({e for f in end for e in f.edges}) + end
bmesh.ops.bisect_plane(bm, geom=geom, plane_co=(T_SEAM, 0, 0), plane_no=(1, 0, 0))
aft = [f for f in bm.faces if near_boom_end(f) and f.calc_center_median().x > T_SEAM]
assert aft, 'nothing of the boom aft of the thruster seam'
bmesh.ops.delete(bm, geom=aft, context='FACES_ONLY')
rim = [e for e in bm.edges if len(e.link_faces) == 1 and all(abs(v.co.x - T_SEAM) < 1e-4 for v in e.verts)]
bulkhead = bmesh.ops.holes_fill(bm, edges=rim, sides=0)['faces']
if len(bulkhead) != 1:
    raise RuntimeError(f'the boom end closed with {len(bulkhead)} faces, not one bulkhead')
bm.to_mesh(me)
bm.free()
normals = body.modifiers.new('normals', 'DATA_TRANSFER')
normals.object = uncut
normals.use_loop_data = True
normals.data_types_loops = {'CUSTOM_NORMAL'}
normals.loop_mapping = 'POLYINTERP_NEAREST'
bpy.ops.object.select_all(action='DESELECT')
body.select_set(True)
bpy.context.view_layer.objects.active = body
bpy.ops.object.modifier_apply(modifier=normals.name)
bpy.data.objects.remove(uncut)
me.materials.clear()
for m in (SKIN, ACCENT, METAL):
    me.materials.append(m)
proj = me.uv_layers.new(name='proj')
proj_view = me.attributes.new('proj_view', 'INT', 'FACE')     # which atlas view each face reads, for check_atlas_fit.py
VIEW_IDS = {'left': 0, 'right': 1, 'roof': 2, 'belly': 3}
for f in me.polygons:
    c, n = Vector(f.center), f.normal
    f.use_smooth = True
    tail_surface = c.x > 18.5 and abs(c.y) > 1.15                      # horizontal stabiliser and the endplate fins: navy
    f.material_index = (2 if n.x < -LEADING_EDGE_FACING else 1) if tail_surface else 0
    # Each face takes the view it faces most; roof and belly are one colour each, so a face the paint line crosses, or that
    # sits on the line's other side, takes a side view instead, where the line is drawn.
    corners = [me.vertices[i].co for i in f.vertices]
    plan_ok = all((v.z >= L.navy_top(v.x)) == (n.z > 0) for v in corners)
    if abs(n.y) >= abs(n.z):
        view, axis = ('right' if n.y > 0 else 'left'), 2
    elif plan_ok:
        view, axis = ('roof' if n.z > 0 else 'belly'), 1
    else:
        view, axis = ('right' if c.y > 0 else 'left'), 2
    for li in f.loop_indices:
        v = me.vertices[me.loops[li].vertex_index].co
        u, w = L.atlas_px(view, v.x, v[axis])
        proj.data[li].uv = (u / L.ATLAS_W, 1.0 - w / L.ATLAS_H)
    proj_view.data[f.index].value = VIEW_IDS[view] if f.material_index == 0 else -1
skin_ref = body.copy()               # the skin as projected, kept in the .blend (never rendered or exported) for the fit check
skin_ref.data = me.copy()
skin_ref.name = 'skin_ref'
skin_ref.hide_render = True
scene.collection.objects.link(skin_ref)
skin = BVHTree.FromObject(body, bpy.context.evaluated_depsgraph_get())


def on_skin(origin, direction):
    """Where a ray from outside meets the skin, and the skin's normal there."""
    loc, nrm, _, _ = skin.ray_cast(Vector(origin), Vector(direction))
    if loc is None:
        raise RuntimeError(f'no skin along {direction} from {origin}')
    return loc, nrm


def cylinder(a, b, r, mat, verts=24, caps='NGON'):
    a, b = Vector(a), Vector(b)
    bpy.ops.mesh.primitive_cylinder_add(radius=r, depth=(b - a).length, location=(a + b) / 2, vertices=verts, end_fill_type=caps)
    ob = bpy.context.active_object
    ob.rotation_euler = (b - a).to_track_quat('Z', 'Y').to_euler()
    ob.data.materials.append(mat)
    for p in ob.data.polygons:
        p.use_smooth = len(p.vertices) == 4      # the sides; a cap smoothed into them shades like a blob
    return ob


def box(centre, size, mat, rot=(0, 0, 0)):
    bpy.ops.mesh.primitive_cube_add(size=1, location=centre, scale=size, rotation=rot)
    ob = bpy.context.active_object
    ob.data.materials.append(mat)
    bpy.ops.object.modifier_add(type='BEVEL')
    ob.modifiers['Bevel'].width = min(size) * 0.3
    ob.modifiers['Bevel'].segments = 2
    return ob


def sphere(centre, r, mat):
    bpy.ops.mesh.primitive_uv_sphere_add(radius=r, location=centre, segments=24, ring_count=12)
    ob = bpy.context.active_object
    ob.data.materials.append(mat)
    for p in ob.data.polygons:
        p.use_smooth = True
    return ob


def ring(centre, major, minor, mat, rot):
    bpy.ops.mesh.primitive_torus_add(major_radius=major, minor_radius=minor, major_segments=24, minor_segments=8,
                                     location=centre, rotation=rot)
    ob = bpy.context.active_object
    ob.data.materials.append(mat)
    for p in ob.data.polygons:
        p.use_smooth = True
    return ob


def tube(points, r, mat):
    """A bent pipe through the points, its bends rounded by a ball each."""
    pts = [Vector(p) for p in points]
    return [cylinder(a, b, r, mat, verts=8) for a, b in zip(pts, pts[1:])] + [sphere(p, r, mat) for p in pts[1:-1]]


fittings = []


def thruster_radius(x):
    """The cones' outer radius, tapering from the seam to the end."""
    return T_R0 + (T_R1 - T_R0) * (x - T_SEAM) / (T_END - T_SEAM)


def angle_off(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def revolve(name, x0, x1, radius, mat, cuts=(), extra_angles=(), caps=(False, False)):
    """A surface of revolution about the thruster axis, angles from +Y toward up; a quad goes where it lies in none of
    cuts, each (x0, x1, centre deg, half-angle deg). Faces point away from the axis; caps close the ends."""
    xs = sorted({x0, x1, *[x0 + (x1 - x0) * i / THRUSTER_STATIONS for i in range(THRUSTER_STATIONS)],
                 *[c for cut in cuts for c in cut[:2] if x0 < c < x1]})
    angles = sorted({round(360.0 * j / THRUSTER_SEGMENTS, 6) for j in range(THRUSTER_SEGMENTS)}
                    | {round(a % 360.0, 6) for a in extra_angles})
    n = len(angles)
    verts = [(x, radius(x) * math.cos(math.radians(a)), T_AXIS_Z + radius(x) * math.sin(math.radians(a))) for x in xs for a in angles]
    faces = []
    for i in range(len(xs) - 1):
        xm = (xs[i] + xs[i + 1]) / 2
        for j in range(n):
            am = (angles[j] + ((angles[(j + 1) % n] - angles[j]) % 360.0) / 2) % 360.0
            if any(c0 < xm < c1 and angle_off(am, centre) < half for c0, c1, centre, half in cuts):
                continue
            faces.append((i * n + j, i * n + (j + 1) % n, (i + 1) * n + (j + 1) % n, (i + 1) * n + j))
    if caps[0]:
        faces.append(tuple(reversed(range(n))))
    if caps[1]:
        faces.append(tuple(range((len(xs) - 1) * n, len(xs) * n)))
    ob = bpy.data.objects.new(name, bpy.data.meshes.new(name))
    ob.data.from_pydata(verts, [], faces)
    ob.data.materials.append(mat)
    for poly in ob.data.polygons:
        poly.use_smooth = len(poly.vertices) == 4
    scene.collection.objects.link(ob)
    return ob


# The thruster's fixed half: the inner cone with its two openings, turning vanes across them, the dark duct behind.
openings = [(T_CUT[0], T_CUT[1], T_LEFT, T_ALPHA), (T_CUT[0], T_CUT[1], T_RIGHT, T_ALPHA)]
inner_r = lambda x: thruster_radius(x) - THRUSTER_GAP
fittings.append(revolve('thruster_inner', T_SEAM, T_END - THRUSTER_GAP, inner_r, METAL, cuts=openings,
                        extra_angles=(T_LEFT - T_ALPHA, T_LEFT + T_ALPHA, T_RIGHT - T_ALPHA, T_RIGHT + T_ALPHA), caps=(False, True)))
fittings.append(revolve('thruster_duct', T_CUT[0] - 0.05, T_CUT[1] + 0.05,
                        lambda x: inner_r(x) - THRUSTER_VANE_DEPTH - 0.03, GRILLE, caps=(True, True)))
vane_count = int((T_CUT[1] - T_CUT[0]) / THRUSTER_VANE_PITCH)
for centre in (T_LEFT, T_RIGHT):
    arc = [centre - T_ALPHA + 2.0 * T_ALPHA * k / 16 for k in range(17)]
    for i in range(vane_count):
        x = T_CUT[0] + (T_CUT[1] - T_CUT[0]) * (i + 0.5) / vane_count
        r_out, r_in = inner_r(x) - 0.01, inner_r(x) - THRUSTER_VANE_DEPTH
        sector = [(x, r * math.cos(math.radians(a)), T_AXIS_Z + r * math.sin(math.radians(a)))
                for r, a in [(r_out, a) for a in arc] + [(r_in, a) for a in reversed(arc)]]
        vane = bpy.data.objects.new('vane', bpy.data.meshes.new('vane'))
        vane.data.from_pydata(sector, [], [tuple(range(len(sector)))])
        vane.data.materials.append(METAL)
        scene.collection.objects.link(vane)
        vane.modifiers.new('thick', 'SOLIDIFY').thickness = THRUSTER_VANE_T
        fittings.append(vane)
# The exhaust where NASA's VSP puts it: an oval on the pylon's aft face, left (Rotor_1_pylon sub-surface SS_ELLIP_0).
fittings.append(cylinder(EXHAUST_FROM, EXHAUST_TO, EXHAUST_R, HOT, verts=32, caps='NOTHING'))
fittings.append(cylinder(EXHAUST_FROM, EXHAUST_FROM + (EXHAUST_TO - EXHAUST_FROM) * 0.3, EXHAUST_R * 0.95, GRILLE, verts=32))
# Air data: a pitot boom off the nose, and its base.
fittings.append(cylinder((-10.2, 0, -6.55), (-12.6, 0, -6.55), 0.045, METAL, verts=16))
fittings.append(cylinder((-10.35, 0, -6.55), (-10.9, 0, -6.55), 0.11, DARK, verts=16))
# Antennas, set on the skin: a GPS puck on top, a radar altimeter under the belly.
puck, puck_n = on_skin((-3.5, 0, 20), (0, 0, -1))
fittings.append(cylinder(puck - puck_n * 0.04, puck + puck_n * 0.08, 0.26, DARK, verts=32))
radalt, radalt_n = on_skin((0.8, 0, -20), (0, 0, 1))
fittings.append(box(radalt + radalt_n * 0.04, (0.6, 0.45, 0.12), DARK))
# A VHF whip under the belly, raked aft.
whip_base, _ = on_skin((WHIP_X, 0, -20), (0, 0, 1))
rake = math.radians(WHIP_RAKE_DEG)
fittings.append(cylinder(whip_base + Vector((0, 0, 0.05)), whip_base - Vector((0, 0, 0.1)), 0.07, DARK, verts=16))
fittings.append(cylinder(whip_base, whip_base + Vector((math.sin(rake), 0, -math.cos(rake))) * WHIP_LEN, 0.02, DARK, verts=8))
# Mechanics' handholds under the engine cowling (the kick-in steps below them are in the atlas), each side.
for s in (-1, 1):
    ends = [on_skin((x, s * 20, HANDHOLD_Z), (0, -s, 0)) for x in HANDHOLD_X]
    tips = [p + n * HANDHOLD_STANDOFF for p, n in ends]
    fittings += [cylinder(p, t, 0.035, STEEL, verts=12) for (p, _), t in zip(ends, tips)]
    fittings += tube(tips, 0.035, STEEL) + [sphere(t, 0.035, STEEL) for t in tips]
# Tie-down rings under the belly, fore and aft, each side.
for x, y in TIE_DOWNS:
    for s in (-1, 1):
        p, _ = on_skin((x, s * y, -20), (0, 0, 1))
        fittings.append(box(p - Vector((0, 0, 0.03)), (0.16, 0.05, 0.1), STRUT))
        fittings.append(ring(p - Vector((0, 0, 0.13)), 0.07, 0.018, STEEL, (math.pi / 2, 0, 0)))

# The NOTAR inlet's screen: wires each way over the atlas's dark recess, laid on the skin ray by ray, so they follow the
# trough NASA's VSP top has there instead of a painted grid stretched over it; a frame round the recess takes their ends.
SCREEN_PITCH, SCREEN_WIRE = 0.08, 0.01   # ft between wires, and each one's half width, chosen
SCREEN_FRAME = 0.02                      # ft, the frame's half width: twice a wire's, so the wires' ends sink into it
SCREEN_STEP, SCREEN_INSET = 0.05, 0.04   # ft between the points a wire follows the skin by, and the frame in from the recess's edge
SCREEN_EDGE_HALVINGS = 30                # where a wire meets the frame, to far below a micron
in_x0, in_y0, in_x1, in_y1 = L.NOTAR_INLET
frame_r = L.NOTAR_INLET_CORNER - SCREEN_INSET   # the frame's corner radius round the recess's corner centres
corner_x0, corner_y0 = in_x0 + L.NOTAR_INLET_CORNER, in_y0 + L.NOTAR_INLET_CORNER
corner_x1, corner_y1 = in_x1 - L.NOTAR_INLET_CORNER, in_y1 - L.NOTAR_INLET_CORNER


def in_inlet(x, y):
    """Inside the frame: within its corner radius of the rectangle the recess's corner centres span."""
    return math.hypot(x - min(max(x, corner_x0), corner_x1), y - min(max(y, corner_y0), corner_y1)) <= frame_r


def skin_under(x, y):
    hit, _, _, _ = skin.ray_cast(Vector((x, y, 20.0)), Vector((0, 0, -1)))
    if hit is None:
        raise RuntimeError(f'the inlet screen finds no skin under ({x:.2f}, {y:.2f})')
    return hit


def screen_wires():
    """One mesh: a square-section wire per line and the frame round them, each lifted its own half width off the skin."""
    verts, faces, up = [], [], Vector((0, 0, 1))

    def wire(points, half):
        base = len(verts)
        for i, p in enumerate(points):
            along = (points[min(i + 1, len(points) - 1)] - points[max(i - 1, 0)]).normalized()
            side = along.cross(up).normalized()
            out = side.cross(along).normalized()
            centre = p + out * half
            verts.extend(centre + side * (s * half) + out * (o * half) for s, o in ((1, 0), (0, 1), (-1, 0), (0, -1)))
        for i in range(len(points) - 1):
            for k in range(4):
                a, b = base + 4 * i + k, base + 4 * i + (k + 1) % 4
                faces.append((a, b, b + 4, a + 4))

    def edge(inside, outside):
        """Where the segment from a point inside the frame to one outside it crosses it."""
        for _ in range(SCREEN_EDGE_HALVINGS):
            mid = ((inside[0] + outside[0]) / 2, (inside[1] + outside[1]) / 2)
            inside, outside = (mid, outside) if in_inlet(*mid) else (inside, mid)
        return inside

    def lay(line):
        """A line's points inside the frame, its ends exactly on it; the recess is convex, so they are one run."""
        inside = [i for i, p in enumerate(line) if in_inlet(*p)]
        if len(inside) < 2:
            return
        first, last = inside[0], inside[-1]
        ends = [edge(line[first], line[first - 1])], [edge(line[last], line[last + 1])]
        wire([skin_under(x, y) for x, y in ends[0] + line[first:last + 1] + ends[1]], SCREEN_WIRE)

    def frame():
        """Four quarter arcs round the corner centres and the straight sides between them, closed."""
        corners = ((corner_x1, corner_y1, 0), (corner_x0, corner_y1, 90), (corner_x0, corner_y0, 180), (corner_x1, corner_y0, 270))
        arc_n = max(2, math.ceil(math.pi / 2 * frame_r / SCREEN_STEP))
        loop = []
        for i, (cx, cy, a0) in enumerate(corners):
            arc = [(cx + frame_r * math.cos(math.radians(a0 + 90 * k / arc_n)), cy + frame_r * math.sin(math.radians(a0 + 90 * k / arc_n)))
                   for k in range(arc_n + 1)]
            nx, ny, na = corners[(i + 1) % 4]
            (sx, sy), (ex, ey) = arc[-1], (nx + frame_r * math.cos(math.radians(na)), ny + frame_r * math.sin(math.radians(na)))
            side_n = max(1, math.ceil(math.hypot(ex - sx, ey - sy) / SCREEN_STEP))
            loop += arc + [(sx + (ex - sx) * k / side_n, sy + (ey - sy) * k / side_n) for k in range(1, side_n)]
        wire([skin_under(x, y) for x, y in loop + loop[:1]], SCREEN_FRAME)

    span = lambda a0, a1, step: [a0 + (a1 - a0) * k / round((a1 - a0) / step) for k in range(round((a1 - a0) / step) + 1)]
    for y in span(in_y0, in_y1, SCREEN_PITCH)[1:-1]:
        lay([(x, y) for x in span(in_x0, in_x1, SCREEN_STEP)])
    for x in span(in_x0, in_x1, SCREEN_PITCH)[1:-1]:
        lay([(x, y) for y in span(in_y0, in_y1, SCREEN_STEP)])
    frame()
    ob = bpy.data.objects.new('inlet_screen', bpy.data.meshes.new('inlet_screen'))
    ob.data.from_pydata(verts, [], faces)
    ob.data.materials.append(STEEL)
    for poly in ob.data.polygons:
        poly.use_smooth = True
    scene.collection.objects.link(ob)
    print('inlet screen', len(verts), 'vertices')
    return ob


fittings.append(screen_wires())


HUB_CAP_PROUD = 0.03                # ft the hub cap stands past the wheel's side, chosen


def wheel(centre, width):
    """Tyre, rim, hub cap and its bolts, turning about Y."""
    c, y = Vector(centre), Vector((0, 1, 0))
    minor = width / 2 * 0.95
    bpy.ops.mesh.primitive_torus_add(major_radius=WHEEL_R - minor, minor_radius=minor, major_segments=72, minor_segments=20,
                                     location=c, rotation=(math.pi / 2, 0, 0))
    tyre = bpy.context.active_object
    tyre.data.materials.append(RUBBER)
    for poly in tyre.data.polygons:
        poly.use_smooth = True
    rim_r = WHEEL_R - 2 * minor + 0.05
    parts = [tyre, cylinder(c - y * width * 0.42, c + y * width * 0.42, rim_r, METAL, verts=48),
             cylinder(c - y * (width * 0.5 + HUB_CAP_PROUD), c + y * (width * 0.5 + HUB_CAP_PROUD), 0.11, STEEL, verts=32)]
    for side in (-1, 1):
        for i in range(6):
            a = math.radians(60 * i)
            b = c + Vector((math.cos(a) * 0.19, side * (width * 0.42 + 0.005), math.sin(a) * 0.19))
            parts.append(cylinder(b, b + y * side * 0.035, 0.025, STEEL, verts=8))
    return parts


def strut_fairing(socket, axis, out, room):
    """A fairing round a strut where it leaves the skin: its wide edge laid on the skin, ray by ray along the strut, so it
    follows whatever curve the skin has there; its narrow edge on the strut, out along the axis, at least DROP past the
    lowest skin under it and short of room, the strut's length out of the skin."""
    radius = min(STRUT_FAIRING_R, STRUT_R + STRUT_FAIRING_FLARE * room)
    across = axis.orthogonal().normalized()
    other = axis.cross(across)
    dirs = [across * math.cos(2 * math.pi * i / STRUT_FAIRING_VERTS) + other * math.sin(2 * math.pi * i / STRUT_FAIRING_VERTS)
            for i in range(STRUT_FAIRING_VERTS)]
    wide = []
    for d in dirs:
        hit, _, _, _ = skin.ray_cast(socket + d * radius + axis * STRUT_FAIRING_RAY, -axis)
        if hit is None:
            raise RuntimeError(f'the fairing round the strut at {tuple(socket)} finds no skin toward {tuple(d)}')
        wide.append(hit - axis * STRUT_FAIRING_SUNK)
    out = max(out, max((w - socket).dot(axis) for w in wide) + STRUT_FAIRING_DROP)
    if out >= room:
        raise RuntimeError(f'the skin under the fairing at {tuple(socket)} falls past the {room:.2f} ft of strut out of it')
    print('strut fairing at', tuple(round(c, 2) for c in socket), 'reaches', round(out, 3), 'ft of', round(room, 3))
    narrow = [socket + axis * out + d * STRUT_R for d in dirs]
    n = STRUT_FAIRING_VERTS
    ob = bpy.data.objects.new('strut_fairing', bpy.data.meshes.new('strut_fairing'))
    ob.data.from_pydata(wide + narrow, [], [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)])
    ob.data.materials.append(STRUT)
    for poly in ob.data.polygons:
        poly.use_smooth = True
    scene.collection.objects.link(ob)
    return ob


def oleo(top, bottom):
    """An oleo strut: the cylinder above, the chrome piston below, a collar where one goes into the other, and the torque
    link in front that keeps the piston from turning."""
    top, bottom = Vector(top), Vector(bottom)
    joint = top.lerp(bottom, 0.58)
    axis = (bottom - top).normalized()
    fwd = Vector((-1, 0, 0))
    fwd = (fwd - axis * fwd.dot(axis)).normalized()
    upper, lower = top.lerp(bottom, 0.42) + fwd * 0.15, top.lerp(bottom, 0.94) + fwd * 0.08
    knee = (upper + lower) / 2 + fwd * 0.25
    up = -axis
    # Where the strut goes into the skin: up its axis from the wheel end, outside, to the first face. From its top, which
    # sits on the belly, a ray slips through and meets the pylon's underside inside the fuselage.
    socket, normal, _, _ = skin.ray_cast(bottom, up)
    if socket is None:
        raise RuntimeError(f'the strut up from {tuple(bottom)} meets no skin')
    outward = normal if normal.dot(axis) > 0 else -normal
    print('strut enters the skin at', tuple(round(c, 2) for c in socket), 'normal', tuple(round(c, 2) for c in outward))
    room = (bottom - socket).dot(axis)
    exit_parts = [strut_fairing(socket, axis, min(STRUT_FAIRING_L, STRUT_FAIRING_SHARE * room), room)]
    return [cylinder(socket + up * STRUT_SOCKET, joint, STRUT_R, STRUT), cylinder(top.lerp(bottom, 0.5), bottom, 0.10, CHROME),
            cylinder(joint - axis * 0.05, joint + axis * 0.05, 0.2, STRUT)] + tube((upper, knee, lower), 0.04, STRUT) + \
        [sphere(knee, 0.055, METAL)] + exit_parts


# Nose gear: a fork round the wheel, its crown under the strut.
y_axis = Vector((0, 1, 0))
crown = FRONT_WHEEL + Vector((0, 0, WHEEL_R + 0.16))
NOSE_LEG_R, NOSE_FORK_GAP = 0.06, 0.04   # ft, chosen: the fork's legs, and their clearance off the hub caps
NOSE_LUG_R, NOSE_LUG_HALF = 0.10, 0.06   # ft, chosen: the round lug each leg ends in, the axle through it
fork_off = FRONT_W / 2 + HUB_CAP_PROUD + NOSE_FORK_GAP + NOSE_LEG_R
fittings += wheel(FRONT_WHEEL, FRONT_W)
fittings += oleo(strut_tops['front'], crown)
NOSE_CROWN_R = 0.08                      # ft, chosen: the crown's cross bar the legs hang from
fittings.append(cylinder(crown - y_axis * fork_off, crown + y_axis * fork_off, NOSE_CROWN_R, STRUT))
for side in (-1, 1):
    foot = FRONT_WHEEL + y_axis * side * fork_off
    fittings.append(cylinder(crown + y_axis * side * fork_off, foot, NOSE_LEG_R, STRUT))
    fittings.append(cylinder(foot - y_axis * NOSE_LUG_HALF, foot + y_axis * NOSE_LUG_HALF, NOSE_LUG_R, STRUT))
axle_half = fork_off + NOSE_LUG_HALF + 0.02             # its ends just past the lugs
fittings.append(cylinder(FRONT_WHEEL - y_axis * axle_half, FRONT_WHEEL + y_axis * axle_half, 0.05, STEEL, verts=12))
TOW_BOX, TOW_RING_R, TOW_RING_T = (0.14, 0.12, 0.1), 0.07, 0.025  # ft, chosen
TOW_SHANK_L, TOW_SHANK_R = 0.05, 0.035                            # ft, chosen: the eye bolt's shank out of the fitting
TOW_EMBED = 0.02                                                  # ft the fitting sits into the crown bar, chosen
x_axis = Vector((1, 0, 0))
tow = crown - x_axis * (NOSE_CROWN_R + TOW_BOX[0] / 2 - TOW_EMBED)  # the tow fitting on the crown bar's front
fittings.append(box(tow, TOW_BOX, STRUT))
# An eye bolt: its ring upright in the fore-aft plane on a shank screwed into the fitting's front face.
eye =tow - x_axis * (TOW_BOX[0] / 2 + TOW_SHANK_L + TOW_RING_R)
fittings.append(cylinder(tow - x_axis * (TOW_BOX[0] / 2 - TOW_SHANK_R), eye + x_axis * TOW_RING_R, TOW_SHANK_R, STEEL, verts=12))
fittings.append(ring(eye, TOW_RING_R, TOW_RING_T, STEEL, (math.pi / 2, 0, 0)))
# Main gear: each strut ends in a forged axle fitting on the wheel's inboard side, the axle through it into the hub.
AXLE_FITTING = (0.26, 0.26, 0.40)   # ft fore-aft, across, along the strut, chosen: wider than the 0.2 ft piston it takes
AXLE_FITTING_BELOW = 0.12           # ft of it below the axle, chosen
AXLE_GAP = 0.06                     # ft between it and the hub cap, chosen: the strut leans 9 deg outward, its corner nearer
for side, zone in ((-1, 'left'), (1, 'right')):
    centre = Vector((MAIN_WHEEL.x, side * MAIN_WHEEL.y, MAIN_WHEEL.z))
    inboard = centre - y_axis * side * (MAIN_W / 2 + HUB_CAP_PROUD + AXLE_GAP + AXLE_FITTING[1] / 2)
    top = strut_tops[zone]
    fittings += wheel(centre, MAIN_W)
    fittings += oleo(top, inboard)
    along = (inboard - top).normalized()
    fittings.append(box(inboard - along * (AXLE_FITTING[2] / 2 - AXLE_FITTING_BELOW), AXLE_FITTING, STRUT,
                        along.to_track_quat('Z', 'Y').to_euler()))
    fittings.append(cylinder(inboard, centre, 0.06, STEEL, verts=12))
# The stationary half of the swashplate sits on the shaft; the turning half is the rotor's.
shaft = Vector((-math.sin(math.radians(SHAFT_TILT_DEG)), 0, math.cos(math.radians(SHAFT_TILT_DEG))))   # its top leans forward
fittings.append(cylinder(shaft * -1.30, shaft * -1.16, 1.05, METAL, verts=48))

bpy.ops.object.select_all(action='DESELECT')
for ob in fittings:
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob
    for mod in list(ob.modifiers):
        bpy.ops.object.modifier_apply(modifier=mod.name)
body.select_set(True)
bpy.context.view_layer.objects.active = body
bpy.ops.object.join()
body = bpy.context.active_object
me = body.data

# ---------------------------------------------------------------- one unwrap, then bake everything onto it
bake_uv = me.uv_layers.new(name='bake')
me.uv_layers.active = bake_uv
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.uv.smart_project(angle_limit=math.radians(UNWRAP_ANGLE_DEG), island_margin=UNWRAP_MARGIN)
bpy.ops.object.mode_set(mode='OBJECT')

scene.render.engine = 'CYCLES'
prefs = bpy.context.preferences.addons['cycles'].preferences
prefs.compute_device_type = 'OPTIX'
prefs.get_devices()
for d in prefs.devices:
    d.use = d.type == 'OPTIX'
print('cycles devices', [(d.name, d.type, d.use) for d in prefs.devices])
scene.cycles.device = 'GPU'
world = bpy.data.worlds.new('sky')
world.use_nodes = True
scene.world = world
world.light_settings.distance = AO_DISTANCE_FT
scene.render.bake.margin = BAKE_MARGIN_PX
mats = list(me.materials)


def bake(name, kind, non_colour, samples, emit_from=None):
    """Bakes one pass into a new image; emit_from(principled) names the input to bake as emission, for channels Cycles has no pass for."""
    img = bpy.data.images.new(name, BAKE_PX, BAKE_PX, alpha=False)
    if non_colour:
        img.colorspace_settings.name = 'Non-Color'
    scene.cycles.samples = samples
    temp = {}
    for m in mats:
        nt = m.node_tree
        t = nt.nodes.new('ShaderNodeTexImage')
        t.image = img
        nt.nodes.active = t
        temp[m] = [t]
        if emit_from:
            src = nt.nodes['Principled BSDF'].inputs[emit_from]
            e = nt.nodes.new('ShaderNodeEmission')
            if src.is_linked:
                nt.links.new(src.links[0].from_socket, e.inputs['Color'])
            else:
                val = src.default_value
                e.inputs['Color'].default_value = tuple(val) if hasattr(val, '__len__') else (val, val, val, 1.0)
            nt.links.new(e.outputs['Emission'], nt.nodes['Material Output'].inputs['Surface'])
            temp[m].append(e)
    bpy.ops.object.select_all(action='DESELECT')
    body.select_set(True)
    bpy.context.view_layer.objects.active = body
    bpy.ops.object.bake(type=kind, uv_layer='bake', normal_space='TANGENT', use_clear=True)
    for m, nodes in temp.items():
        nt = m.node_tree
        for n in nodes:
            nt.nodes.remove(n)
        nt.links.new(nt.nodes['Principled BSDF'].outputs['BSDF'], nt.nodes['Material Output'].inputs['Surface'])
    img.filepath_raw = os.path.join(out_dir, f'orca_{name}.png')
    img.file_format = 'PNG'
    img.save()
    print('baked', name)
    return img


colour = bake('basecolor', 'EMIT', False, FLAT_SAMPLES, 'Base Color')
normal = bake('normal', 'NORMAL', True, FLAT_SAMPLES)
ao = bake('ao', 'AO', True, AO_SAMPLES)
rough = bake('roughness', 'EMIT', True, FLAT_SAMPLES, 'Roughness')
metal = bake('metallic', 'EMIT', True, FLAT_SAMPLES, 'Metallic')


def channel(img):
    a = np.empty(BAKE_PX * BAKE_PX * 4, np.float32)
    img.pixels.foreach_get(a)
    return a.reshape(-1, 4)[:, 0]


orm = bpy.data.images.new('orm', BAKE_PX, BAKE_PX, alpha=False)
orm.colorspace_settings.name = 'Non-Color'
packed = np.ones((BAKE_PX * BAKE_PX, 4), np.float32)
packed[:, 0], packed[:, 1], packed[:, 2] = channel(ao), channel(rough), channel(metal)
orm.pixels.foreach_set(packed.ravel())
orm.filepath_raw = os.path.join(out_dir, 'orca_orm.png')
orm.file_format = 'PNG'
orm.save()

# The one material the glb carries; glTF reads occlusion off a "glTF Material Output" group, its ORM packing is glTF's own.
out_mat = bpy.data.materials.new('orca')
out_mat.use_nodes = True
nt = out_mat.node_tree
p = nt.nodes['Principled BSDF']
for img in (colour, normal, orm):
    img.reload()
col_t, nrm_t, orm_t = (nt.nodes.new('ShaderNodeTexImage') for _ in range(3))
col_t.image, nrm_t.image, orm_t.image = colour, normal, orm
nt.links.new(col_t.outputs['Color'], p.inputs['Base Color'])
sep = nt.nodes.new('ShaderNodeSeparateColor')
nt.links.new(orm_t.outputs['Color'], sep.inputs['Color'])
nt.links.new(sep.outputs['Green'], p.inputs['Roughness'])
nt.links.new(sep.outputs['Blue'], p.inputs['Metallic'])
nm = nt.nodes.new('ShaderNodeNormalMap')
nt.links.new(nrm_t.outputs['Color'], nm.inputs['Color'])
nt.links.new(nm.outputs['Normal'], p.inputs['Normal'])
gltf_out = bpy.data.node_groups.new('glTF Material Output', 'ShaderNodeTree')
gltf_out.interface.new_socket(name='Occlusion', in_out='INPUT', socket_type='NodeSocketFloat')
grp = nt.nodes.new('ShaderNodeGroup')
grp.node_tree = gltf_out
nt.links.new(sep.outputs['Red'], grp.inputs['Occlusion'])
me.materials.clear()
me.materials.append(out_mat)
for f in me.polygons:
    f.material_index = 0
while len(me.uv_layers) > 1:      # looked up afresh each time: a removal shifts the others' references
    me.uv_layers.remove(next(l for l in me.uv_layers if l.name != 'bake'))
assert me.uv_layers[0].name == 'bake'

# ---------------------------------------------------------------- rotor: one levelled blade six times, hub, grips, linkage
src = load('rotor')
bm_src = bmesh.new()
bm_src.from_mesh(src.data)
bpy.data.objects.remove(src)   # only its geometry is used; the export's own rotor must not stay in the scene


def piece(keep):
    """A copy of the source rotor keeping only the faces whose every vertex passes keep(co)."""
    bm = bm_src.copy()
    bmesh.ops.delete(bm, geom=[f for f in bm.faces if not all(keep(v.co) for v in f.verts)], context='FACES')
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context='VERTS')
    return bm


def radius(co):
    return math.hypot(co.x, co.y)


hub_bm = piece(lambda co: radius(co) < HUB_R + 0.05)
blade_bm = piece(lambda co: radius(co) > BLADE_ROOT_R - 0.05 and abs(math.degrees(math.atan2(co.y, co.x))) < 25)
# VSP turns the whole rotor about its origin, hub and blades alike: both are levelled back, so they spin about one axis.
level = Matrix.Rotation(math.radians(SHAFT_TILT_DEG), 4, 'Y')
bmesh.ops.transform(hub_bm, matrix=level, verts=hub_bm.verts)
bmesh.ops.transform(blade_bm, matrix=level, verts=blade_bm.verts)
tip_r = max(radius(v.co) for v in blade_bm.verts)

rotor_me = bpy.data.meshes.new('rotor')
hub_bm.to_mesh(rotor_me)
rotor_me.materials.append(ACCENT)
rotor = bpy.data.objects.new('rotor', rotor_me)
scene.collection.objects.link(rotor)
blade_me = bpy.data.meshes.new('blade')
blade_bm.to_mesh(blade_me)
blade0 = bpy.data.objects.new('blade0', blade_me)
scene.collection.objects.link(blade0)
# The blade keeps the export's own grid (qsmr_split.vspscript), cut down by nothing: a collapse decimation left slivers
# that streaked its highlights. Edges folded past BLADE_SHARP_DEG (the trailing edge) stay sharp, so smoothing does not
# shade across them.
sharp_bm = bmesh.new()
sharp_bm.from_mesh(blade_me)
for e in sharp_bm.edges:
    if e.is_manifold and math.degrees(e.calc_face_angle(0.0)) > BLADE_SHARP_DEG:
        e.smooth = False
sharp_bm.to_mesh(blade_me)
sharp_bm.free()
print('blade faces', len(blade_me.polygons))
blade_me.materials.append(BLADE)
blade_me.materials.append(TIP)
for f in blade_me.polygons:
    f.material_index = 1 if radius(Vector(f.center)) > tip_r - 1.2 else 0
    f.use_smooth = True
blades = [blade0]
for k in range(1, 6):
    ob = bpy.data.objects.new(f'blade{k}', blade_me.copy())
    scene.collection.objects.link(ob)
    ob.rotation_euler = (0, 0, math.radians(60 * k))
    blades.append(ob)

SWASH_Z, SWASH_R, SWASH_T = -1.05, 1.0, 0.12   # the turning swashplate, under the hub drum (whose bottom is -0.75)
HORN_R, HORN_LEAD = 1.85, 0.42                   # the pitch horn leaves the grip here, toward the leading edge
parts = []
for k in range(6):
    a = math.radians(60 * k)
    d = Vector((math.cos(a), math.sin(a), 0))
    parts.append(cylinder(d * (HUB_R - 0.25) + Vector((0, 0, BLADE_ROOT_Z)), d * (BLADE_ROOT_R + 0.3) + Vector((0, 0, BLADE_ROOT_Z)),
                          0.17, METAL))
    t = Vector((-math.sin(a), math.cos(a), 0))       # toward the leading edge
    horn_root = d * HORN_R + Vector((0, 0, BLADE_ROOT_Z))
    horn_tip = horn_root + t * HORN_LEAD - Vector((0, 0, 0.06))
    parts.append(cylinder(horn_root, horn_tip, 0.06, METAL, verts=12))
    link_foot = Vector((horn_tip.x, horn_tip.y, SWASH_Z + SWASH_T / 2))
    parts.append(cylinder(link_foot, horn_tip, 0.045, STEEL, verts=12))      # the pitch link, straight down
    for end in (link_foot, horn_tip):
        parts.append(sphere(end, 0.075, METAL))                               # its rod ends
    lug_r = Vector((horn_tip.x, horn_tip.y, 0)).length
    lug_dir = Vector((horn_tip.x, horn_tip.y, 0)).normalized()
    parts.append(cylinder(Vector((0, 0, SWASH_Z)) + lug_dir * (SWASH_R - 0.1), Vector((0, 0, SWASH_Z)) + lug_dir * (lug_r + 0.05),
                          0.07, METAL, verts=12))                             # the swashplate's arm out to the link
    behind = a - math.radians(DAMPER_SPREAD_DEG)                              # the lead-lag damper, hub to the grip's trailing side
    d_hub = Vector((math.cos(behind), math.sin(behind), 0)) * (HUB_R - 0.1) + Vector((0, 0, BLADE_ROOT_Z))
    d_grip = d * (BLADE_ROOT_R - 0.1) - t * 0.24 + Vector((0, 0, BLADE_ROOT_Z))
    parts += [cylinder(d_hub, d_grip, 0.13, STRUT, verts=16),         # an elastomeric damper: one thick body, no exposed rod
              sphere(d_hub, 0.08, METAL), sphere(d_grip, 0.08, METAL)]
hub_top = BVHTree.FromBMesh(hub_bm)
for i in range(HUB_BOLTS):
    a = 2 * math.pi * i / HUB_BOLTS
    p, n, _, _ = hub_top.ray_cast(Vector((HUB_BOLT_R * math.cos(a), HUB_BOLT_R * math.sin(a), 20)), Vector((0, 0, -1)))
    if p is None:
        raise RuntimeError(f'hub bolt {i} at r {HUB_BOLT_R} misses the hub')
    parts.append(cylinder(p - n * 0.02, p + n * 0.06, 0.06, STEEL, verts=6))
parts.append(cylinder((0, 0, SWASH_Z - SWASH_T / 2), (0, 0, SWASH_Z + SWASH_T / 2), SWASH_R, METAL, verts=48))   # turning half

bpy.ops.object.select_all(action='DESELECT')
for ob in blades + parts:
    ob.select_set(True)
rotor.select_set(True)
bpy.context.view_layer.objects.active = rotor
bpy.ops.object.join()
rotor = bpy.context.active_object
for f in rotor.data.polygons:
    f.use_smooth = True
print('rotor faces', len(rotor.data.polygons), 'tip radius', round(tip_r, 2))

# The thruster's turning half: the outer cone with its one cutout, drawn at neutral pedal; it turns about its axis by
# fcs/notar/sleeve-angle-deg less T_NEUTRAL_DEG (positive from +Y toward up, i.e. about +X).
pivot = Vector((T_SEAM, 0.0, T_AXIS_Z))
thruster = revolve('thruster', T_SEAM + 0.005, T_END, thruster_radius, ACCENT,
                   cuts=[(T_CUT[0], T_CUT[1], T_NEUTRAL_DEG, T_BETA)],
                   extra_angles=(T_NEUTRAL_DEG - T_BETA, T_NEUTRAL_DEG + T_BETA), caps=(False, True))
for v in thruster.data.vertices:
    v.co -= pivot
thruster.location = pivot
thruster.modifiers.new('wall', 'SOLIDIFY').thickness = THRUSTER_SLEEVE_T
bpy.ops.object.select_all(action='DESELECT')
thruster.select_set(True)
bpy.context.view_layer.objects.active = thruster
bpy.ops.object.modifier_apply(modifier='wall')
print('thruster', len(thruster.data.polygons), 'faces, cutout half-angle', round(T_BETA, 2), 'neutral', round(T_NEUTRAL_DEG, 2))

for ob in (body, rotor, thruster):
    bpy.ops.object.select_all(action='DESELECT')
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob
    ob.location = ob.location * FOOT
    ob.scale = (FOOT, FOOT, FOOT)
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)

# ---------------------------------------------------------------- render with the baked material
bpy.ops.mesh.primitive_plane_add(size=200, location=(0, 0, GROUND_Z * FOOT))
ground = bpy.context.active_object
ground.data.materials.append(material('ground', srgb(170, 172, 168), rough=0.9))
bpy.ops.object.light_add(type='SUN', rotation=(math.radians(50), math.radians(10), math.radians(-40)))
bpy.context.active_object.data.energy = 4.0
world.node_tree.nodes['Background'].inputs['Color'].default_value = (0.55, 0.65, 0.8, 1.0)
world.node_tree.nodes['Background'].inputs['Strength'].default_value = 0.8
scene.render.engine = 'BLENDER_EEVEE_NEXT'
scene.render.resolution_x, scene.render.resolution_y = 1600, 1000
cam = bpy.data.objects.new('cam', bpy.data.cameras.new('cam'))
scene.collection.objects.link(cam)
scene.camera = cam
views = {
    '34': ((5.0, 0, -4.5), -35, 18, 19.0),
    'right34': ((5.0, 0, -4.5), 35, 18, 19.0),
    'front34': ((5.0, 0, -4.5), -150, 15, 17.0),
    'nose': ((-5.0, 0, -5.0), -120, 10, 5.5),
    'roof': ((2.0, 0, -1.5), -60, 55, 5.0),
    'boom': ((14.0, 0, -3.3), 70, 8, 5.0),
}
for tag, (centre, az, el, dist) in views.items():
    c = Vector(centre) * FOOT
    a, e = math.radians(az), math.radians(el)
    cam.location = c + Vector((dist * math.cos(e) * math.cos(a), dist * math.cos(e) * math.sin(a), dist * math.sin(e)))
    cam.rotation_euler = (c - cam.location).to_track_quat('-Z', 'Y').to_euler()
    scene.render.filepath = os.path.join(out_dir, f'orca_{tag}.png')
    bpy.ops.render.render(write_still=True)
    print('rendered', tag)

bpy.data.objects.remove(ground)
for name, ob in (('body', body), ('rotor', rotor), ('thruster', thruster)):
    bpy.ops.object.select_all(action='DESELECT')
    ob.select_set(True)
    bpy.ops.export_scene.gltf(filepath=os.path.join(out_dir, f'orca_{name}.glb'), use_selection=True, export_format='GLB')
    print('exported', name, len(ob.data.polygons), 'faces')
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(out_dir, 'orca.blend'))
