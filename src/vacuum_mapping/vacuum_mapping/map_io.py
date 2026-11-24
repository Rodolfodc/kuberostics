"""Reading and writing nav2_map_server maps (`<name>.yaml` + `<name>.pgm`) as `Raster` (003-R33, R34).

WHY: kept apart from the cores so the algorithms never touch the disk (D-18). Only the trinary format
that `map_saver` writes is supported: binary PGM (P5), 8 bit, map yaw 0.
"""
from __future__ import annotations

import os
from typing import Dict, Tuple

import numpy as np
import yaml

from .core.types import FREE, OCCUPIED, UNKNOWN, Raster


UNKNOWN_PIXEL = 205  # grey written by map_saver for unknown cells


def _read_pgm(path: str) -> np.ndarray:
    """Pixels of a binary 8-bit PGM as a (height, width) uint8 array, top row first."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:  # magic, width, height, maxval
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while end < len(data) and not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    pos += 1  # exactly one whitespace byte separates the header from the pixels
    if tokens[0] != b'P5' or int(tokens[3]) != 255:
        raise ValueError('%s: only binary 8-bit PGM (P5, maxval 255) is supported' % path)
    width, height = int(tokens[1]), int(tokens[2])
    pixels = np.frombuffer(data, dtype=np.uint8, count=width * height, offset=pos)
    return pixels.reshape(height, width)


def _write_pgm(path: str, pixels: np.ndarray) -> None:
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (pixels.shape[1], pixels.shape[0]))
        f.write(np.ascontiguousarray(pixels, dtype=np.uint8).tobytes())


def read_map(yaml_path: str) -> Raster:
    """Load a map saved by `map_saver`; `image` is resolved relative to the YAML file.

    Thresholds of the file apply, except that in non-negated maps the pixel value 205 is always UNKNOWN:
    with the usual free_thresh 0.25, (255 - 205) / 255 = 0.196 would otherwise read as free.
    """
    with open(yaml_path, 'r', encoding='utf-8') as f:
        meta: Dict = yaml.safe_load(f)
    origin = meta['origin']
    if len(origin) > 2 and abs(float(origin[2])) > 1e-9:
        raise ValueError('%s: map yaw %s is not supported' % (yaml_path, origin[2]))
    image = meta['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    pixels = _read_pgm(image).astype(np.float64)
    occupancy = pixels / 255.0 if meta.get('negate', 0) else (255.0 - pixels) / 255.0
    cells = np.full(pixels.shape, UNKNOWN, dtype=np.int8)
    cells[occupancy < float(meta.get('free_thresh', 0.25))] = FREE
    cells[occupancy > float(meta.get('occupied_thresh', 0.65))] = OCCUPIED
    if not meta.get('negate', 0):
        cells[pixels == UNKNOWN_PIXEL] = UNKNOWN  # map_saver's unknown grey is never free, whatever free_thresh says
    # PGM rows run top to bottom; Raster row 0 is the lowest y.
    return Raster(np.flipud(cells), float(meta['resolution']), float(origin[0]), float(origin[1]))


def write_map(raster: Raster, yaml_path: str, image_name: str = '') -> Tuple[str, str]:
    """Write `raster` as PGM + YAML next to each other; returns the two paths. Overwrites without asking:
    the caller (regeneration script) is responsible for never pointing it at the committed raster (003-R33)."""
    base = os.path.splitext(yaml_path)[0]
    image_name = image_name or os.path.basename(base) + '.pgm'
    pgm_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image_name)
    pixels = np.full(raster.cells.shape, 205, dtype=np.uint8)  # map_saver's unknown grey
    pixels[raster.cells == FREE] = 254
    pixels[raster.cells == OCCUPIED] = 0
    _write_pgm(pgm_path, np.flipud(pixels))
    meta = {
        'image': image_name,
        'mode': 'trinary',
        'resolution': float(raster.resolution),
        'origin': [float(raster.origin_x), float(raster.origin_y), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    with open(yaml_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    return yaml_path, pgm_path
