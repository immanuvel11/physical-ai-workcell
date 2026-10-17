"""Class-agnostic tabletop segmentation from a registered RGB-D frame.

No trained detector: anything that rises above the table is an object. Each segment gets a
metric 3D box (centre, size, yaw), a colour name and a coarse shape from its geometry.
Large footprints are receptacles: hollow ones are bins, flat ones are trays; parts inside a bin
or on a tray are split out by height (and colour, for bins).

Pure numpy/OpenCV so it can be unit-tested on saved frames.
"""
import colorsys
import math
from dataclasses import dataclass, field

import cv2
import numpy as np

MIN_HEIGHT = 0.008          # above the support surface to count as an object (m)
MAX_HEIGHT = 0.30           # ignore things taller than this (arms, people)
MIN_PIXELS = 25
RECEPTACLE_MIN_SIDE = 0.12  # footprint side above which a segment is a receptacle
MAX_GRASP_WIDTH = 0.064


@dataclass
class Segment:
    shape: str                  # box | cylinder | bin | tray
    category: str               # part | receptacle | obstacle
    color: str
    rgb: tuple
    center: tuple               # x, y, z (box centre, world)
    size: tuple                 # extent along the segment's own x, y, and height
    yaw: float
    pixels: int
    bbox: tuple                 # u0, v0, u1, v1 in the image
    support_z: float = 0.0
    children: list = field(default_factory=list)

    @property
    def graspable(self):
        return self.category == 'part' and min(self.size[0], self.size[1]) <= MAX_GRASP_WIDTH


def color_name(rgb, ref_v=None):
    """Colour word. ref_v = brightness of the table under the current light: whites and greys are
    judged relative to it, so a white part stays 'white' when the light dims."""
    r, g, b = (c / 255.0 for c in rgb)
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    rel = v / ref_v if ref_v else v / 0.72          # 0.72 ~ table brightness under nominal light
    if rel < 0.3:
        return 'black'
    if s < 0.18:
        return 'white' if rel > 1.12 else 'gray'
    deg = h * 360
    for limit, name in ((15, 'red'), (40, 'orange'), (70, 'yellow'), (160, 'green'),
                        (200, 'cyan'), (255, 'blue'), (290, 'purple'), (335, 'pink'), (361, 'red')):
        if deg < limit:
            return name
    return 'red'


def backproject(depth, K, R, t):
    """World XYZ image (H, W, 3) from a planar depth image, intrinsics K and camera->world (R, t)."""
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    z = depth
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    cam = np.stack([x, y, z], axis=-1)
    return cam @ R.T + t


def robot_mask(xyz, segments, valid):
    """Pixels within a capsule around any robot link segment [(p0, p1, radius), ...]."""
    mask = np.zeros(valid.shape, bool)
    idx = np.nonzero(valid)
    pts = xyz[idx]
    hit = np.zeros(len(pts), bool)
    for p0, p1, radius in segments:
        d = p1 - p0
        L2 = float(d @ d) or 1e-9
        s = np.clip(((pts - p0) @ d) / L2, 0.0, 1.0)
        closest = p0 + s[:, None] * d
        hit |= np.sum((pts - closest) ** 2, axis=1) < radius * radius
    mask[idx[0][hit], idx[1][hit]] = True
    return mask


def _rect(points_xy):
    (cx, cy), (w, h), ang = cv2.minAreaRect(points_xy.astype(np.float32))
    yaw = math.radians(ang)
    # normalise so that size_x >= size_y and yaw in [-pi/2, pi/2)
    if w < h:
        w, h, yaw = h, w, yaw + math.pi / 2
    yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2
    return cx, cy, w, h, yaw


def _bbox(vs, us):
    return int(us.min()), int(vs.min()), int(us.max()), int(vs.max())


def _part(rgb, xyz, comp, support_z, ref_v=None):
    """Build a part/obstacle Segment from a boolean pixel mask resting on support_z."""
    vs, us = np.nonzero(comp)
    pts = xyz[vs, us]
    top = float(np.percentile(pts[:, 2], 95))
    height = max(top - support_z, 0.005)
    topmask = pts[:, 2] > top - 0.012                      # top face: best for shape, footprint and colour
    top_pts = pts[topmask]
    # Footprint from the top face (perspective makes the side walls of tall parts spill outward).
    cx, cy, w, h, yaw = _rect(top_pts[:, :2] if len(top_pts) >= 20 else pts[:, :2])
    # Shape from the top face: a disc fills ~78% of its bounding square, a box ~100%.
    tv, tu = vs[topmask], us[topmask]
    (_, _), (pw, ph), _ = cv2.minAreaRect(np.stack([tu, tv], 1).astype(np.float32))
    fill = len(top_pts) / max((pw + 1) * (ph + 1), 1.0)   # +1: rect spans pixel centres, not edges
    aspect = w / max(h, 1e-3)
    shape = 'cylinder' if fill < 0.86 and aspect < 1.25 else 'box'
    if shape == 'cylinder':
        w = h = max(w, h)
    col = np.median(rgb[tv, tu], axis=0)
    category = 'part' if min(w, h) <= MAX_GRASP_WIDTH and height < 0.12 else 'obstacle'
    return Segment(shape=shape, category=category, color=color_name(col, ref_v), rgb=tuple(int(c) for c in col),
                   center=(cx, cy, support_z + height / 2), size=(w, h, height), yaw=yaw,
                   pixels=len(pts), bbox=_bbox(vs, us), support_z=support_z)


def _parts_in(mask, rgb, xyz, support_z, ref_v=None):
    """Parts in a content mask; close small gaps first so one part is not split in two."""
    mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)).astype(bool)
    return [_part(rgb, xyz, c, support_z, ref_v) for c in _components(mask, 15)]


def _components(mask, min_pixels=MIN_PIXELS):
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    return [labels == i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= min_pixels]


def _inside_rect(xyz, cx, cy, w, h, yaw, margin):
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = xyz[..., 0] - cx, xyz[..., 1] - cy
    lx, ly = c * dx - s * dy, s * dx + c * dy
    return (np.abs(lx) <= w / 2 - margin) & (np.abs(ly) <= h / 2 - margin)


def segment(rgb, xyz, valid, table_bounds, robot):
    """Return a list of Segments. rgb (H,W,3) uint8, xyz world points, valid bool mask,
    table_bounds (x0, x1, y0, y1), robot bool mask of pixels to ignore."""
    x0, x1, y0, y1 = table_bounds
    z = xyz[..., 2]
    on_table = valid & (xyz[..., 0] > x0) & (xyz[..., 0] < x1) & (xyz[..., 1] > y0) & (xyz[..., 1] < y1)
    occ = on_table & ~robot & (z > MIN_HEIGHT) & (z < MAX_HEIGHT)
    occ = cv2.morphologyEx(occ.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)).astype(bool)
    bare = on_table & ~robot & (z < 0.004)
    ref_v = float(np.median(rgb[bare].max(axis=1))) / 255.0 if bare.sum() > 500 else None

    # Anything touching the robot is not trusted as an object (an unusual arm pose can leave parts of
    # the robot outside its mask; adding those to the collision world would deadlock the planner).
    near_robot = cv2.dilate(robot.astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)
    out, loose = [], []
    for comp in _components(occ):
        if (comp & near_robot).sum() > 5:
            continue
        vs, us = np.nonzero(comp)
        pts = xyz[vs, us]
        cx, cy, w, h, yaw = _rect(pts[:, :2])
        if min(w, h) < RECEPTACLE_MIN_SIDE:
            loose.append(comp)      # decided after receptacles are known (could be inside one)
            continue
        # Receptacle. Hollow (interior well below the rim) => bin, else flat => tray.
        rim_band = comp & ~_inside_rect(xyz, cx, cy, w, h, yaw, margin=0.015)
        rim = float(np.median(z[rim_band])) if rim_band.any() else float(np.percentile(pts[:, 2], 50))
        interior = _inside_rect(xyz, cx, cy, w, h, yaw, margin=0.02) & on_table & ~robot
        low = interior & (z < 0.5 * rim)
        hollow = low.sum() / max(interior.sum(), 1)
        if hollow > 0.35:
            seg = _receptacle(rgb, xyz, comp, 'bin', cx, cy, w, h, yaw, ref_v=ref_v)
            floor = 0.006
            # Contents by geometry only: the walls are excluded by position (margin), so anything inside
            # them and above the floor is a part - including a part of the same colour as the bin.
            inner = _inside_rect(xyz, cx, cy, w, h, yaw, margin=0.012) & on_table & ~robot
            content = inner & (z > floor + MIN_HEIGHT)
            seg.children = _parts_in(content, rgb, xyz, floor, ref_v)
        else:
            zs = pts[:, 2]
            hist, edges = np.histogram(zs, bins=np.arange(0.0, MAX_HEIGHT, 0.004))
            surface = float(edges[np.argmax(hist)] + 0.002)          # dominant height = tray top
            base = comp & (np.abs(z - surface) < 0.006)
            bvs, bus = np.nonzero(base)
            cx, cy, w, h, yaw = _rect(xyz[bvs, bus][:, :2]) if len(bvs) > 20 else (cx, cy, w, h, yaw)
            seg = _receptacle(rgb, xyz, base if len(bvs) > 20 else comp, 'tray', cx, cy, w, h, yaw, top=surface,
                              ref_v=ref_v)
            content = comp & (z > surface + MIN_HEIGHT)
            seg.children = _parts_in(content, rgb, xyz, surface, ref_v)
        out.append(seg)
    # Free-standing parts; ones whose centre is inside a receptacle are already its children.
    for comp in loose:
        part = _part(rgb, xyz, comp, support_z=0.0, ref_v=ref_v)
        px, py = part.center[0], part.center[1]
        if not any(_inside_rect(np.array([px, py, 0.0]), r.center[0], r.center[1], r.size[0], r.size[1], r.yaw, 0.0)
                   for r in out):
            out.append(part)
    return out


def _receptacle(rgb, xyz, comp, kind, cx, cy, w, h, yaw, top=None, ref_v=None):
    vs, us = np.nonzero(comp)
    pts = xyz[vs, us]
    top = float(np.percentile(pts[:, 2], 95)) if top is None else top
    col = np.median(rgb[vs, us], axis=0)
    return Segment(shape=kind, category='receptacle', color=color_name(col, ref_v), rgb=tuple(int(c) for c in col),
                   center=(cx, cy, top / 2), size=(w, h, top), yaw=yaw, pixels=len(pts), bbox=_bbox(vs, us))


def flatten(segments):
    """Receptacles followed by every part (including parts inside receptacles)."""
    flat = []
    for s in segments:
        flat.append(s)
        flat.extend(s.children)
    return flat


def shape_word(s):
    if s.shape == 'cylinder':
        return 'cylinder'
    if s.shape in ('bin', 'tray'):
        return s.shape
    w, h, z = s.size
    if w / max(h, 1e-3) < 1.25 and abs(z - w) < 0.012:
        return 'cube'
    return 'block' if w / max(h, 1e-3) >= 1.25 else 'box'


def annotate(rgb, segments, labels=None):
    """BGR debug image with numbered boxes."""
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    for i, s in enumerate(flatten(segments)):
        u0, v0, u1, v1 = s.bbox
        color = (0, 200, 255) if s.category == 'receptacle' else (0, 255, 0) if s.category == 'part' else (0, 0, 255)
        cv2.rectangle(img, (u0, v0), (u1, v1), color, 1)
        text = f'{i + 1}:{labels[i] if labels else s.color + " " + shape_word(s)}'
        cv2.putText(img, text, (u0, max(v0 - 4, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 3)
        cv2.putText(img, text, (u0, max(v0 - 4, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)
    return img
