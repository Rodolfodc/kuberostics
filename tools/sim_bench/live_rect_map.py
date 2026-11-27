#!/usr/bin/env python3
"""Cuts a small coverage region out of a saved scan map: only the rectangle X0 Y0 X1 Y1 (world metres) stays free, the
rest becomes occupied (local-only). Usage: live_rect_map.py IN_DIR OUT_DIR X0 Y0 X1 Y1 ; IN_DIR from live_map_scan.py"""
import os, sys
import numpy as np
src, out = sys.argv[1], sys.argv[2]; x0, y0, x1, y1 = map(float, sys.argv[3:7])
meta = dict(l.split(': ', 1) for l in open(os.path.join(src, 'room.yaml')).read().splitlines())
ox, oy = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]; res = float(meta['resolution'])
raw = open(os.path.join(src, 'room.pgm'), 'rb').read().split(b'\n', 3); w, h = map(int, raw[1].split())
img = np.flipud(np.frombuffer(raw[3], np.uint8).reshape(h, w)).copy()   # row 0 = min y
new = np.zeros_like(img)
for j in range(h):
    for i in range(w):
        x, y = ox + (i + 0.5) * res, oy + (j + 0.5) * res
        if x0 <= x <= x1 and y0 <= y <= y1 and img[j, i] == 254:
            new[j, i] = 254
os.makedirs(out, exist_ok=True)
open(os.path.join(out, 'room.pgm'), 'wb').write(b'P5\n%d %d\n255\n' % (w, h) + np.flipud(new).tobytes())
open(os.path.join(out, 'room.yaml'), 'w').write(open(os.path.join(src, 'room.yaml')).read())
print('free cells', int((new == 254).sum()), 'area m2', round((new == 254).sum() * res * res, 2))
