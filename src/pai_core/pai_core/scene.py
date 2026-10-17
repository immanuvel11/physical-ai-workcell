"""Scene-graph helpers shared by the world model, skills, planner and benchmark."""
import math

from pai_core.geometry import point_in_rect, quat_yaw

BIN_FLOOR = 0.008        # floor thickness + margin inside a bin (m)
TABLE_TOP_GAP = 0.002    # planning-scene table top sits this far below z=0 (robot base rests on it)
NEAR_DIST = 0.08         # parts closer than this are 'near' each other


def bottom_z(o):
    return o.pose.position.z - o.size.z / 2


def top_z(o):
    return o.pose.position.z + o.size.z / 2


def support_surface_z(receptacle):
    """Height at which a part rests when placed in/on this receptacle."""
    if receptacle.shape == 'bin':
        return bottom_z(receptacle) + BIN_FLOOR
    return top_z(receptacle)


def is_within(part, receptacle, margin=0.01):
    """True if the part's centre lies inside the receptacle footprint and not above its rim."""
    p, r = part.pose.position, receptacle.pose.position
    inside = point_in_rect(p.x, p.y, r.x, r.y, quat_yaw(receptacle.pose.orientation),
                           receptacle.size.x, receptacle.size.y, margin)
    return inside and bottom_z(part) < top_z(receptacle) + 0.02


def compute_relations(objects):
    """List of (subject, predicate, object) triples: in/on receptacles, on the table, near each other."""
    receptacles = [o for o in objects if o.category == 'receptacle']
    parts = [o for o in objects if o.category != 'receptacle']
    rels = []
    for p in parts:
        holder = next((r for r in receptacles if is_within(p, r)), None)
        if holder is not None:
            rels.append((p.id, 'in' if holder.shape == 'bin' else 'on', holder.id))
        else:
            rels.append((p.id, 'on', 'table'))
    for i, a in enumerate(parts):
        for b in parts[i + 1:]:
            pa, pb = a.pose.position, b.pose.position
            if math.hypot(pa.x - pb.x, pa.y - pb.y) < NEAR_DIST:
                rels.append((a.id, 'near', b.id))
    return rels


def object_to_dict(o):
    p = o.pose.position
    return {
        'id': o.id, 'label': o.label, 'color': o.color, 'shape': o.shape, 'category': o.category,
        'position_m': [round(p.x, 3), round(p.y, 3), round(p.z, 3)],
        'yaw_deg': round(math.degrees(quat_yaw(o.pose.orientation))),
        'size_cm': [round(o.size.x * 100, 1), round(o.size.y * 100, 1), round(o.size.z * 100, 1)],
        'graspable': o.graspable,
    }


def scene_to_dict(scene):
    return {
        'objects': [object_to_dict(o) for o in scene.objects],
        'relations': [f'{r.subject} {r.predicate} {r.object}' for r in scene.relations],
        'held_object': scene.held_object,
    }


def find(scene, object_id):
    return next((o for o in scene.objects if o.id == object_id), None)


def scene_graph_from_yaml(scene, gz_positions=None):
    """Ground-truth SceneGraph from a scene YAML (+ live gz positions if given).

    Used to test skills without perception and by the benchmark as the oracle.
    """
    from geometry_msgs.msg import Vector3
    from pai_core.geometry import yaw_quat
    from pai_interfaces.msg import DetectedObject, SceneGraph

    def make(name, category, shape, color_rgb, size_xyz, x, y, yaw):
        o = DetectedObject(id=name, label=name.replace('_', ' '), color=name.split('_')[0],
                           shape=shape, category=category, confidence=1.0)
        o.size = Vector3(x=float(size_xyz[0]), y=float(size_xyz[1]), z=float(size_xyz[2]))
        p = (gz_positions or {}).get(name)
        o.pose.position.x, o.pose.position.y = (p.x, p.y) if p else (float(x), float(y))
        o.pose.position.z = p.z if p else size_xyz[2] / 2
        o.pose.orientation = yaw_quat(yaw)
        o.graspable = category == 'part' and min(size_xyz[0], size_xyz[1]) <= 0.064
        return o

    def extents(o):
        s = o['size']
        return (s[0], s[1], s[2]) if o['shape'] == 'box' else (2 * s[0], 2 * s[0], s[1])

    sg = SceneGraph()
    for f in scene.get('fixtures', []):
        x, y, _, yaw = f['pose']
        sg.objects.append(make(f['name'], 'receptacle', f['type'], f['color'], f['size'], x, y, yaw))
    for key, category in (('objects', 'part'), ('obstacles', 'obstacle')):
        for o in scene.get(key, []):
            if o.get('parked'):
                continue
            x, y, yaw = o['pose']
            sg.objects.append(make(o['name'], category, o['shape'], o['color'], extents(o), x, y, yaw))
    return sg
