# -*- coding: utf-8 -*-
"""
D9b: Figure 6 montage, revision 2 (seek-safe).
Pass 1: sequential full scan -> per-frame presence, empty runs, panel selection.
Pass 2: sequential re-read, caching the chosen frames + fresh inference on them
        (sequential reads only; POS_FRAMES seek on this 4K h264 stream is unreliable).
Outputs: fig6/panel_*.png, fig6/fig6_montage.png, fig6/meta.txt, fig6/DONE.txt
"""
import os
import cv2
import numpy as np
from ultralytics import YOLO

SPARSE = r"C:\deepseek\论文\文件列表\sparse.mp4"
CROP   = r"C:\deepseek\论文\文件列表\vehicle_crop.jpg"
OUTDIR = r"C:\deepseek\论文\文件列表\fig6"
os.makedirs(OUTDIR, exist_ok=True)

CARS     = {2, 5, 7}
MIN_AREA = 0.01
PANEL_W  = 1280

def log(msg):
    print(msg, flush=True)

model = YOLO("yolov8n.pt")

cap = cv2.VideoCapture(SPARSE)
fps = cap.get(cv2.CAP_PROP_FPS) or 57.9
W   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
H   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
n   = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
log(f"video: {W}x{H} fps={fps:.1f} frames={n}")

def rel_boxes(frame):
    """Relevant ego-lane boxes (same filter as d4/d7)."""
    r = model(frame, verbose=False)[0]
    out = []
    for c, (x1, y1, x2, y2), cf in zip(r.boxes.cls.tolist(), r.boxes.xyxy.tolist(), r.boxes.conf.tolist()):
        if int(c) not in CARS:
            continue
        if (x2 - x1) * (y2 - y1) < MIN_AREA * W * H:
            continue
        cx, cy = (x1 + x2) / 2 / W, (y1 + y2) / 2 / H
        if 0.2 < cx < 0.8 and cy > 0.45:
            out.append([x1, y1, x2, y2, float(cf)])
    return out

# ---------- Pass 1: full sequential scan ----------
pres = [False] * n
nb   = [None] * n
idx  = 0
cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
while True:
    ok, frame = cap.read()
    if not ok:
        break
    bx = rel_boxes(frame)
    if bx:
        pres[idx] = True
        nb[idx]   = max(bx, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
    if idx % 400 == 0:
        log(f"scan {idx}/{n}")
    idx += 1
cap.release()
log(f"scan done: present = {sum(pres)}/{n} ({100*sum(pres)/n:.1f}%)")

runs = []
i = 0
while i < n:
    if not pres[i]:
        j = i
        while j < n and not pres[j]:
            j += 1
        runs.append((i, j))
        i = j
    else:
        i += 1
log(f"empty runs: {len(runs)}  max {(max((e-s for s,e in runs), default=0))/fps:.2f} s")

def pick_frames():
    out = {}
    n3 = n // 3
    cand = []
    i = 0
    while i < n:
        if pres[i]:
            j = i
            while j < n and pres[j]:
                j += 1
            cand.append((i, j))
            i = j
        else:
            i += 1
    good = [r0 for r0 in cand if r0[1] - r0[0] >= 30 and n3 <= (r0[0] + r0[1]) // 2 <= 2 * n3]
    if not good:
        good = [r0 for r0 in cand if r0[1] - r0[0] >= 20]
    r0 = max(good, key=lambda r0: r0[1] - r0[0])
    t = (r0[0] + r0[1]) // 2
    out["cont"] = [t - 10, t, t + 10]
    log(f"continuity triple frames {out['cont']} (run [{r0[0]},{r0[1]}) { (r0[1]-r0[0])/fps:.2f}s)")

    mid = [r for r in runs if 25 <= r[1] - r[0] <= 120]
    if not mid:
        mid = [r for r in runs if r[1] - r[0] >= 10]
    mid.sort(key=lambda r: r[1] - r[0], reverse=True)
    e0 = mid[0]
    out["empty"] = (e0[0], e0[1])
    log(f"empty window [{e0[0]},{e0[1]}) { (e0[1]-e0[0])/fps:.2f}s")

    other = [r for r in runs if r != e0]
    other.sort(key=lambda r: r[1] - r[0], reverse=True)
    out["inj"] = other[0][0] if other else (e0[1] + 20)
    log(f"injection frame {out['inj']}")
    return out

sel = pick_frames()
needed = set(sel["cont"]) | {sel["empty"][0], sel["empty"][1], sel["inj"]}

# ---------- Pass 2: sequential re-read, cache + infer only the needed frames ----------
cache = {}          # idx -> (scaled BGR panel, list of boxes)
idx = 0
cap2 = cv2.VideoCapture(SPARSE)
cap2.set(cv2.CAP_PROP_POS_FRAMES, 0)
while True:
    ok, frame = cap2.read()
    if not ok:
        break
    if idx in needed:
        bx = rel_boxes(frame)
        cache[idx] = (frame, bx)
        log(f"captured frame {idx} boxes={len(bx)}")
    idx += 1
cap2.release()

def draw(frame, boxes, injected=False):
    scale = PANEL_W / frame.shape[1]
    Hh = int(frame.shape[0] * scale)
    vis = cv2.resize(frame, (PANEL_W, Hh), interpolation=cv2.INTER_AREA)
    for b in boxes:
        x1, y1, x2, y2, cf = [v * scale for v in b[:4]] + [b[4]]
        color = (0, 0, 255) if injected else (0, 255, 0)
        cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), color, 3)
        lab = "injected" if injected else f"car {cf:.2f}"
        cv2.putText(vis, lab, (int(x1), max(20, int(y1) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)
    return vis

log("rendering panels...")
meta = []
panels = []
for k, t in enumerate(sel["cont"]):
    frame, bx = cache[t]
    meta.append(f"panel {chr(97+k)}: frame {t} ({t/fps:.2f}s) boxes={len(bx)}")
    p = draw(frame, bx)
    panels.append(p)
    cv2.imwrite(os.path.join(OUTDIR, f"panel_{chr(97+k)}.png"), p)

t_d = sel["empty"][0]
fd, bx_d = cache[t_d]
meta.append(f"panel d: empty-window frame {t_d} ({t_d/fps:.2f}s) boxes={len(bx_d)}")
p = draw(fd, [])
panels.append(p)
cv2.imwrite(os.path.join(OUTDIR, "panel_d.png"), p)

t_e = sel["empty"][1]
fe, bx_e = cache[t_e]
meta.append(f"panel e: first detection after window {t_e} ({t_e/fps:.2f}s) boxes={len(bx_e)}")
p = draw(fe, bx_e)
panels.append(p)
cv2.imwrite(os.path.join(OUTDIR, "panel_e.png"), p)

t_f = sel["inj"]
ff, bx_f0 = cache[t_f]
crop = cv2.imread(CROP)
ch, cw = crop.shape[:2]
bw = int(0.12 * W)
bh = int(bw * ch / cw)
bx2, by2 = int(W * 0.62), int(H * 0.76)
bx1, by1 = max(0, bx2 - bw), max(0, by2 - bh)
ff2 = ff.copy()
ff2[by1:by2, bx1:bx2] = cv2.resize(crop, (bx2 - bx1, by2 - by1))
bx_f = rel_boxes(ff2)
det = any(any(max(min(b[2], bx2) - max(b[0], bx1), 0) * max(min(b[3], by2) - max(b[1], by1), 0) > 0 for b in bx_f) for _ in [0])
meta.append(f"panel f: injection frame {t_f} ({t_f/fps:.2f}s) injected=[{bx1},{by1},{bx2},{by2}] detected={det}")
p = draw(ff2, [[bx1, by1, bx2, by2, 1.0]], injected=True)
panels.append(p)
cv2.imwrite(os.path.join(OUTDIR, "panel_f.png"), p)

# ---------- montage ----------
gap = 14
def stack_row(a, b, c):
    return np.hstack([a, np.full((a.shape[0], gap, 3), 255, np.uint8), b,
                      np.full((a.shape[0], gap, 3), 255, np.uint8), c])
row1 = stack_row(panels[0], panels[1], panels[2])
row2 = stack_row(panels[3], panels[4], panels[5])
mont = np.vstack([row1, np.full((gap, row1.shape[1], 3), 255, np.uint8), row2])

for k in range(6):
    x = (k % 3) * (PANEL_W + gap) + 24
    y = (k // 3) * (panels[0].shape[0] + gap) + 58
    cv2.putText(mont, f"({chr(97+k)})", (x, y),
                cv2.FONT_HERSHEY_SIMPLEX, 1.6, (255, 255, 255), 8, cv2.LINE_AA)
    cv2.putText(mont, f"({chr(97+k)})", (x, y),
                cv2.FONT_HERSHEY_SIMPLEX, 1.6, (30, 30, 30), 3, cv2.LINE_AA)

mpath = os.path.join(OUTDIR, "fig6_montage.png")
cv2.imwrite(mpath, mont)
log(f"montage: {mpath} {mont.shape[1]}x{mont.shape[0]}")

with open(os.path.join(OUTDIR, "meta.txt"), "w", encoding="utf-8") as fh:
    fh.write("\n".join(meta) + "\n")
with open(os.path.join(OUTDIR, "DONE.txt"), "w", encoding="utf-8") as fh:
    fh.write("OK\n")
log("DONE")