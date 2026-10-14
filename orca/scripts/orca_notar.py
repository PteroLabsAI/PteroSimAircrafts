"""Reads the Orca's NOTAR sizes from its JSBSim system (Systems/notar.xml), so the mesh and the physics share
one source: the Coanda slot's width (uav_atlas.py) and the thruster's cones, openings and cutout (build_orca.py)."""
import os
import xml.etree.ElementTree as ET


def load(fdm_dir):
    return ET.parse(os.path.join(fdm_dir, 'Systems', 'notar.xml')).getroot()


def prop(root, name):
    found = [float(e.get('value')) for e in root.iter('property') if e.get('value') is not None and (e.text or '').strip() == name]
    if len(found) != 1:
        raise RuntimeError(f'notar.xml: {len(found)} values for {name}')
    return found[0]


def table_max(root, function):
    fn = [e for e in root.iter('fcs_function') if e.get('name') == function]
    if len(fn) != 1:
        raise RuntimeError(f'notar.xml: {len(fn)} functions named {function}')
    rows = [line.split() for line in fn[0].find('.//tableData').text.strip().splitlines()]
    return max(float(r[1]) for r in rows)
