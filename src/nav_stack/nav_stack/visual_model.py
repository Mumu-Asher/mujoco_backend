"""把 URDF 的视觉几何叠加到已有 MuJoCo body 上（只做外观，不参与物理）。

用于在新 MuJoCo 后端里显示 robocon 的 br 小车：物理底盘仍是学长的麦轮模型，
只把 br 的 mesh/box/cylinder 视觉按零位关节的 FK 贴上去，contype/conaffinity=0。
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def _origin(elem):
    T = np.eye(4)
    if elem is None:
        return T
    o = elem.find('origin')
    if o is None:
        return T
    xyz = [float(v) for v in (o.get('xyz') or '0 0 0').split()]
    rpy = [float(v) for v in (o.get('rpy') or '0 0 0').split()]
    T[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    T[:3, 3] = xyz
    return T


def _find_body(root, name):
    for body in root.iter('body'):
        if body.get('name') == name:
            return body
    return None


def add_visual_urdf(root, urdf_path, name, parent_body_name, group='2', hide_prefix='omni/',
                    offset=(0.0, 0.0, 0.0)):
    path = Path(urdf_path).resolve()
    urdf = ET.parse(path).getroot()
    links = {link.get('name'): link for link in urdf.findall('link')}
    parent_of, joint_of = {}, {}
    for joint in urdf.findall('joint'):
        parent_of[joint.find('child').get('link')] = joint.find('parent').get('link')
        joint_of[joint.find('child').get('link')] = joint
    roots = [n for n in links if n not in parent_of]
    if not roots:
        raise ValueError('URDF 没有根 link')

    # 零位关节下的 FK：fixed/revolute/prismatic 都只取 joint origin
    T = {roots[0]: np.eye(4)}
    T[roots[0]][:3, 3] = offset
    stack = [roots[0]]
    while stack:
        p = stack.pop()
        for child, parent in parent_of.items():
            if parent != p:
                continue
            T[child] = T[p] @ _origin(joint_of[child])
            stack.append(child)

    parent = _find_body(root, parent_body_name)
    if parent is None:
        raise ValueError(f'找不到 MuJoCo body: {parent_body_name}')
    asset = root.find('asset')
    if asset is None:
        asset = ET.SubElement(root, 'asset')

    # 隐藏学长的底盘外观（碰撞/物理保持不变；raycast 本来就不含 group 1）
    for geom in parent.iter('geom'):
        gname = geom.get('name', '')
        if gname.startswith(hide_prefix) and '_visual_' in gname:
            geom.set('rgba', '0 0 0 0')

    count = 0
    for link_name, link in links.items():
        if link_name not in T:
            continue
        for idx, visual in enumerate(link.findall('visual')):
            Tw = T[link_name] @ _origin(visual)
            # 用四元数而不是 euler：MuJoCo 的 euler 是角度制且顺序与 scipy 不同，
            # 之前误把弧度当角度传进去，90° 的轮/臂全被压成了 ~1.57°。
            qx, qy, qz, qw = Rotation.from_matrix(Tw[:3, :3]).as_quat()
            attrs = {
                'name': f'{name}/{link_name}_vis_{idx}',
                'pos': ' '.join(f'{v:.6f}' for v in Tw[:3, 3]),
                'quat': f'{qw:.8f} {qx:.8f} {qy:.8f} {qz:.8f}',
                'contype': '0', 'conaffinity': '0', 'group': group,
            }
            color = visual.find('material/color')
            attrs['rgba'] = color.get('rgba') if color is not None else '.6 .6 .6 1'
            geometry = visual.find('geometry')
            if geometry is None:
                continue
            box, cyl, mesh = geometry.find('box'), geometry.find('cylinder'), geometry.find('mesh')
            if box is not None:
                attrs.update(type='box',
                             size=' '.join(str(float(v) / 2) for v in box.get('size').split()))
            elif cyl is not None:
                attrs.update(type='cylinder',
                             size=f'{cyl.get("radius")} {float(cyl.get("length")) / 2}')
            elif mesh is not None:
                fn = mesh.get('filename', '')
                if fn.startswith('package://'):
                    pkg, rel = fn[len('package://'):].split('/', 1)
                    from ament_index_python.packages import get_package_share_directory
                    mp = Path(get_package_share_directory(pkg)) / rel
                elif fn.startswith('file://'):
                    mp = Path(fn[len('file://'):])
                else:
                    mp = path.parent / fn
                if not mp.is_absolute():
                    mp = path.parent / mp
                mesh_name = f'{name}/{link_name}_mesh_{idx}'
                ET.SubElement(asset, 'mesh', name=mesh_name, file=str(mp.resolve()),
                              scale=mesh.get('scale', '1 1 1'))
                attrs.update(type='mesh', mesh=mesh_name)
            else:
                continue
            ET.SubElement(parent, 'geom', **attrs)
            count += 1
    return count
