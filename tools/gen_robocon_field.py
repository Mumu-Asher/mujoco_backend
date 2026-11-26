#!/usr/bin/env python3
"""从 robocon_ground/model.sdf 的 collision 生成 MuJoCo 场地 XML。

用法:
    python3 tools/gen_robocon_field.py [model.sdf] [out.xml]

默认读取仓库自带的 tools/robocon_ground/model.sdf，
输出到 src/nav_stack/worlds/robocon_ground.xml。
"""
import math
import re
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
SDF = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / 'robocon_ground' / 'model.sdf'
OUT = Path(sys.argv[2]) if len(sys.argv) > 2 else HERE.parent / 'src/nav_stack/worlds/robocon_ground.xml'

def color_for(name):
    if name.startswith('Ground'): return '0.62 0.65 0.67 1'
    if name.startswith('Boundary'): return '0.32 0.36 0.40 1'
    if name.startswith('Cube.001') or name.startswith('Cube.006'): return '0.85 0.72 0.45 1'
    if name.startswith('Cube.00') and 'fill' in name: return '0.85 0.72 0.45 1'
    if name == 'Cube': return '0.90 0.55 0.20 1'
    if re.match(r'Cube\.0(0[2-9]|10)$', name): return '0.20 0.50 0.75 1'
    if re.match(r'Cube\.0(1[1-9])$', name): return '0.75 0.25 0.25 1'
    if re.match(r'Cube\.0(2[0-9]|3[0-9]|4[0-3])$', name): return '0.25 0.65 0.35 1'
    if name.startswith('Cylinder'): return '0.55 0.35 0.15 1'
    return '0.70 0.70 0.70 1'

s = SDF.read_text()
root = ET.Element('mujoco', {'model': 'robocon_ground'})
ET.SubElement(root, 'compiler', {'angle': 'radian'})
ET.SubElement(root, 'option', {'timestep': '.001', 'integrator': 'implicitfast', 'cone': 'elliptic'})
vis = ET.SubElement(root, 'visual')
ET.SubElement(vis, 'global', {'offwidth': '1280', 'offheight': '800'})
ET.SubElement(vis, 'headlight', {'ambient': '.4 .4 .4', 'diffuse': '.7 .7 .7'})
asset = ET.SubElement(root, 'asset')
ET.SubElement(asset, 'texture', {'name': 'floor_grid', 'type': '2d', 'builtin': 'checker',
                                 'rgb1': '.69 .74 .76', 'rgb2': '.76 .80 .81', 'width': '512', 'height': '512'})
ET.SubElement(asset, 'material', {'name': 'floor_grid', 'texture': 'floor_grid',
                                  'texrepeat': '11 11', 'reflectance': '.04'})
ET.SubElement(asset, 'texture', {'name': 'sky', 'type': 'skybox', 'builtin': 'gradient',
                                 'rgb1': '.15 .19 .24', 'rgb2': '.4 .48 .55', 'width': '256', 'height': '1536'})
wb = ET.SubElement(root, 'worldbody')
ET.SubElement(wb, 'light', {'pos': '0 -3 8', 'dir': '0 .3 -1', 'directional': 'true'})
# 单一地面，名字必须是 floor（OmniWorld 用它计算轮子载荷/牵引）
ET.SubElement(wb, 'geom', {'name': 'floor', 'type': 'box', 'pos': '0 0 -0.01',
                           'size': '5.5 5.5 0.01', 'rgba': '0.62 0.65 0.67 1'})

count = 0
for m in re.finditer(r'<collision name="([^"]+)">(.*?)</collision>', s, re.S):
    name, body = m.group(1), m.group(2)
    if name.startswith('Ground'):
        continue
    pose = re.search(r'<pose>([^<]*)</pose>', body)
    vals = [float(v) for v in pose.group(1).split()] if pose else [0, 0, 0, 0, 0, 0]
    x, y, z, r, p, yaw = vals
    box = re.search(r'<box>\s*<size>([^<]*)</size>', body, re.S)
    cyl = re.search(r'<cylinder>\s*<radius>([^<]*)</radius>\s*<length>([^<]*)</length>', body, re.S)
    if box:
        sx, sy, sz = [float(v) for v in box.group(1).split()]
        attrs = {'name': name, 'type': 'box',
                 'size': f'{sx/2:.6f} {sy/2:.6f} {sz/2:.6f}',
                 'pos': f'{x:.6f} {y:.6f} {z:.6f}',
                 'rgba': color_for(name)}
        if abs(r) + abs(p) + abs(yaw) > 1e-9:
            attrs['euler'] = f'{r:.6f} {p:.6f} {yaw:.6f}'
    elif cyl:
        rad, length = float(cyl.group(1)), float(cyl.group(2))
        attrs = {'name': name, 'type': 'cylinder',
                 'size': f'{rad:.6f} {length/2:.6f}',
                 'pos': f'{x:.6f} {y:.6f} {z:.6f}',
                 'rgba': color_for(name)}
    else:
        continue
    ET.SubElement(wb, 'geom', **attrs)
    count += 1

OUT.parent.mkdir(parents=True, exist_ok=True)
xml = ET.tostring(root, encoding='unicode')
xml = xml.replace(' />', '/>')
OUT.write_text('<?xml version="1.0" encoding="utf-8"?>\n' + xml + '\n')
print(f'{count} geoms -> {OUT}')
