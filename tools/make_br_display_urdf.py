#!/usr/bin/env python3
"""生成 RViz 用的 br 显示 URDF（不参与物理）。

把 ``models/br/br_sim.urdf`` 转成：
  * mesh 路径改成 ``package://nav_stack/models/br/meshes_sim/...``（随包安装）；
  * 根 link 改名为 ``br_base_link``，并加一个空 ``base_link`` 到它的 fixed joint；
  * 抬高 0.053 m，让 br 的轮子正好落在物理 base_link（地面）上；
  * 去掉 collision（RViz 只看外观）。

物理仿真仍然用学长的 4 轮麦轮底盘，br 只是 RViz 里的显示模型。
"""
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / 'src' / 'nav_stack'
SRC = ROOT / 'models' / 'br' / 'br_sim.urdf'
DST = ROOT / 'urdf' / 'br_display.urdf'
Z_OFFSET = 0.053


def main():
    root = ET.parse(SRC).getroot()

    # 1) mesh 路径 -> package://nav_stack/models/br/...
    for mesh in root.iter('mesh'):
        fn = mesh.get('filename', '')
        if fn.startswith('meshes_sim/'):
            mesh.set('filename', 'package://nav_stack/models/br/' + fn)

    # 2) 找 URDF 根 link（没有 parent joint 的 link）
    children = {j.find('child').get('link') for j in root.findall('joint')}
    links = root.findall('link')
    roots = [l for l in links if l.get('name') not in children]
    if len(roots) != 1:
        raise SystemExit(f'期望一个根 link，实际 {len(roots)} 个')
    root_link = roots[0]
    old_name = root_link.get('name')
    root_link.set('name', 'br_base_link')
    for joint in root.findall('joint'):
        parent = joint.find('parent')
        if parent is not None and parent.get('link') == old_name:
            parent.set('link', 'br_base_link')

    # 3) 去掉 collision（RViz 不需要，也省内存）
    for link in links:
        for col in list(link.findall('collision')):
            link.remove(col)

    # 4) 所有关节冻结在零位：RViz 只做显示，不需要轮子/升降机构随 /joint_states 动。
    #    避免 RViz 对缺失 joint_state 的关节采用非零默认值，导致上部分“错位/分离”。
    for joint in root.findall('joint'):
        joint.set('type', 'fixed')
        for tag in ('axis', 'limit', 'mimic', 'dynamics', 'calibration',
                    'safety_controller'):
            for elem in list(joint.findall(tag)):
                joint.remove(elem)

    # 4) 空 base_link + 抬升 fixed joint
    mounts = [l for l in links if l.get('name') == 'base_link']
    if not mounts:
        base = ET.SubElement(root, 'link', name='base_link')
        # 保持 link 在前面，joint 在后面
        root.remove(base)
        root.insert(list(root).index(links[0]), base)
    joint = ET.SubElement(root, 'joint', name='br_mount', type='fixed')
    ET.SubElement(joint, 'origin', xyz=f'0 0 {Z_OFFSET}', rpy='0 0 0')
    ET.SubElement(joint, 'parent', link='base_link')
    ET.SubElement(joint, 'child', link='br_base_link')

    ET.indent(root)
    DST.write_text(ET.tostring(root, encoding='unicode'))
    print(f'wrote {DST} ({DST.stat().st_size} bytes)')


if __name__ == '__main__':
    main()
