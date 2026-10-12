# -*- coding: utf-8 -*-
"""
D9-assemble: rebuild fig6_montage.png from the six panel PNGs.
All panels are resized to PANEL_W width, labelled (a)-(f), stacked 2 rows x 3 cols.
Usage: python d9_asm.py
"""
import os
import cv2
import numpy as np

OUTDIR = r"C:\deepseek\论文\文件列表\fig6"
PANEL_W = 1280
GAP = 14

def load_panel(name):
    p = os.path.join(OUTDIR, name)
    # unicode-safe read (cv2.imread fails on non-ASCII paths on Windows)
    im = cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)
    if im is None:
        raise SystemExit(f"missing {name}")
    scale = PANEL_W / im.shape[1]
    Hh = int(im.shape[0] * scale)
    return cv2.resize(im, (PANEL_W, Hh), interpolation=cv2.INTER_AREA)

names = ["panel_a.png", "panel_b.png", "panel_c.png",
         "panel_d.png", "panel_e.png", "panel_f.png"]
panels = [load_panel(n) for n in names]

def stack_row(a, b, c):
    w = np.full((a.shape[0], GAP, 3), 255, np.uint8)
    return np.hstack([a, w, b, w, c])

row1 = stack_row(panels[0], panels[1], panels[2])
row2 = stack_row(panels[3], panels[4], panels[5])
white = np.full((GAP, row1.shape[1], 3), 255, np.uint8)
mont = np.vstack([row1, white, row2])

for k in range(6):
    x = (k % 3) * (PANEL_W + GAP) + 24
    y = (k // 3) * (panels[0].shape[0] + GAP) + 58
    cv2.putText(mont, f"({chr(97 + k)})", (x, y),
                cv2.FONT_HERSHEY_SIMPLEX, 1.6, (255, 255, 255), 8, cv2.LINE_AA)
    cv2.putText(mont, f"({chr(97 + k)})", (x, y),
                cv2.FONT_HERSHEY_SIMPLEX, 1.6, (30, 30, 30), 3, cv2.LINE_AA)

mpath = os.path.join(OUTDIR, "fig6_montage.png")
# unicode-safe write
ok, buf = cv2.imencode(".png", mont)
if not ok:
    raise SystemExit("imencode failed")
buf.tofile(mpath)
print(f"montage: {mpath} {mont.shape[1]}x{mont.shape[0]}")