#!/usr/bin/env python3
"""抽稀 br 的 STL（>20万面 MuJoCo 不接受），写成二进制 STL 到 meshes_sim/，并生成 br_sim.urdf。

注意：精简仓库没有提交 models/br/meshes/ 原始网格（约 43MB），
运行仿真不需要重新生成；只有需要从原始 CAD 重新抽稀时，才把 meshes/ 放回再执行本脚本。
"""
from pathlib import Path
import open3d as o3d

root = Path(__file__).resolve().parents[1] / 'src/nav_stack/models/br'
src = root / 'meshes'
dst = root / 'meshes_sim'
dst.mkdir(exist_ok=True)
target = 50000
for path in sorted(src.rglob('*')):
    if path.suffix.lower() != '.stl' or not path.is_file():
        continue
    rel = path.relative_to(src)
    out = dst / rel
    out.parent.mkdir(parents=True, exist_ok=True)
    mesh = o3d.io.read_triangle_mesh(str(path))
    n = len(mesh.triangles)
    if n == 0:
        print('skip empty', rel)
        continue
    if n > target:
        mesh = mesh.simplify_quadric_decimation(target)
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(str(out), mesh, write_ascii=False)
    print(f'{rel}: {n} -> {len(mesh.triangles)} triangles')

urdf = (root / 'br.urdf').read_text()
(root / 'br_sim.urdf').write_text(urdf.replace('meshes/', 'meshes_sim/'))
print('wrote br_sim.urdf')
