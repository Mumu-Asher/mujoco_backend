#!/usr/bin/env python3
"""Port the official plugin's Euler(0, zenith - pi/2, azimuth) * unit X.

精简仓库没有内置 third_party/livox_laser_simulation（约 173MB）；
生成好的 src/nav_stack/scan_patterns/mid360_directions.npy 已提交，运行不需要本脚本。
如需重新生成，先克隆 https://github.com/Livox-SDK/livox_laser_simulation，
再用 --csv 指定 mid360.csv（或放到默认路径）：
    python3 scripts/convert_mid360_pattern.py --csv /path/to/mid360.csv
"""
import argparse
from pathlib import Path
import numpy as np
root = Path(__file__).resolve().parents[1]
ap = argparse.ArgumentParser()
ap.add_argument('--csv', default=str(root/'third_party/livox_laser_simulation/scan_mode/mid360.csv'))
args = ap.parse_args()
csv = Path(args.csv)
if not csv.is_file():
    raise SystemExit(f'找不到 {csv}；请从 livox_laser_simulation 仓库获取 mid360.csv 并用 --csv 指定')
rows = np.loadtxt(csv, delimiter=',', skiprows=1)
azimuth = np.deg2rad(rows[:, 1])
elevation = np.deg2rad(90. - rows[:, 2])
directions = np.column_stack((np.cos(elevation)*np.cos(azimuth),
                             np.cos(elevation)*np.sin(azimuth), np.sin(elevation)))
output = root/'src/nav_stack/scan_patterns/mid360_directions.npy'
output.parent.mkdir(parents=True, exist_ok=True)
np.save(output, directions.astype(np.float32))
print(len(rows), 'directions saved to', output)
