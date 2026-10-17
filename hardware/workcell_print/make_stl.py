#!/usr/bin/env python3
"""Printable workcell kit for the SO-101 version of the project (bins, kit tray, test parts).

    python3 make_stl.py            # writes the .stl files next to this script

Sizes are scaled to the SO-101's reach (comfortable top-down picks ~12-24 cm from its base) and to
its gripper (keep parts <= 30 mm on the narrow side). Print in PLA, 0.2 mm layers, 15-20 % infill,
no supports needed. Print each part in the colour named in its file name: perception names objects
by colour, so a "red cube" must really be red.
All dimensions are millimetres. Change them below and re-run to regenerate.
"""
import math
import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

BIN = dict(outer=(80.0, 80.0, 35.0), wall=2.4, floor=1.6)     # two of these: blue and green
TRAY = dict(outer=(110.0, 80.0, 8.0), rim=2.0, floor=2.0)      # kit tray with a low rim, grey
PARTS = {
    'red_cube_25mm': ('box', (25, 25, 25)),
    'yellow_block_35x18x18mm': ('box', (35, 18, 18)),
    'blue_bolt_d16_h35mm': ('cylinder', (16, 35)),
    'green_nut_d24_h12mm_hole10': ('ring', (24, 10, 12)),
    'orange_box_30x20x20mm': ('box', (30, 20, 20)),           # the "unseen object" for novelty tests
}


# ------------------------------------------------------------------------------------- meshes
def box(x0, y0, z0, x1, y1, z1):
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], float)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
         (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return [v[list(t)] for t in f]


def cylinder(d, h, segments=64):
    r = d / 2
    tris = []
    for i in range(segments):
        a0, a1 = 2 * math.pi * i / segments, 2 * math.pi * (i + 1) / segments
        p0, p1 = (r * math.cos(a0), r * math.sin(a0)), (r * math.cos(a1), r * math.sin(a1))
        b0, b1, t0, t1 = (*p0, 0), (*p1, 0), (*p0, h), (*p1, h)
        tris += [np.array([(0, 0, 0), b1, b0]), np.array([(0, 0, h), t0, t1]),
                 np.array([b0, b1, t1]), np.array([b0, t1, t0])]
    return tris


def ring(d_out, d_in, h, segments=64):
    ro, ri = d_out / 2, d_in / 2
    tris = []
    for i in range(segments):
        a0, a1 = 2 * math.pi * i / segments, 2 * math.pi * (i + 1) / segments
        c0, s0, c1, s1 = math.cos(a0), math.sin(a0), math.cos(a1), math.sin(a1)
        o0b, o1b, o0t, o1t = (ro * c0, ro * s0, 0), (ro * c1, ro * s1, 0), (ro * c0, ro * s0, h), (ro * c1, ro * s1, h)
        i0b, i1b, i0t, i1t = (ri * c0, ri * s0, 0), (ri * c1, ri * s1, 0), (ri * c0, ri * s0, h), (ri * c1, ri * s1, h)
        tris += [np.array([o0b, o1b, o1t]), np.array([o0b, o1t, o0t]),        # outer wall
                 np.array([i0b, i1t, i1b]), np.array([i0b, i0t, i1t]),        # inner wall
                 np.array([o0t, o1t, i1t]), np.array([o0t, i1t, i0t]),        # top
                 np.array([o0b, i1b, o1b]), np.array([o0b, i0b, i1b])]        # bottom
    return tris


def open_box(sx, sy, sz, wall, floor):
    """Bin / tray: floor plus four walls, as overlapping boxes (slicers merge them fine)."""
    return (box(0, 0, 0, sx, sy, floor)
            + box(0, 0, 0, wall, sy, sz) + box(sx - wall, 0, 0, sx, sy, sz)
            + box(0, 0, 0, sx, wall, sz) + box(0, sy - wall, 0, sx, sy, sz))


def write_stl(path, tris, name):
    with open(path, 'wb') as f:
        f.write(name.encode()[:80].ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0])
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack('<12fH', *n, *t[0], *t[1], *t[2], 0))


def main():
    out = []
    sx, sy, sz = BIN['outer']
    for colour in ('blue', 'green'):
        name = f'{colour}_bin_{int(sx)}x{int(sy)}x{int(sz)}mm'
        out.append((name, open_box(sx, sy, sz, BIN['wall'], BIN['floor'])))
    tx, ty, tz = TRAY['outer']
    out.append((f'grey_kit_tray_{int(tx)}x{int(ty)}x{int(tz)}mm', open_box(tx, ty, tz, TRAY['rim'], TRAY['floor'])))
    for name, (kind, dims) in PARTS.items():
        tris = box(0, 0, 0, *dims) if kind == 'box' else cylinder(*dims) if kind == 'cylinder' else ring(*dims)
        out.append((name, tris))
    for name, tris in out:
        write_stl(os.path.join(HERE, f'{name}.stl'), tris, name)
        print(f'{name}.stl  ({len(tris)} triangles)')


if __name__ == '__main__':
    main()
