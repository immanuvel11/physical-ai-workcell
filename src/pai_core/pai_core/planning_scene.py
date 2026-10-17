"""SceneGraph -> MoveIt collision objects (so planned motions avoid the table, bins and other parts)."""
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

from pai_core.scene import TABLE_TOP_GAP

TABLE = (0.35, 0.0, 1.2, 1.4)   # centre x, y and size x, y of the table top (matches make_world.py)
WALL = 0.006


def _pose(x, y, z, q=None):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    if q is None:
        p.orientation.w = 1.0
    else:
        p.orientation = q
    return p


def _box(dx, dy, dz):
    return SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(dx), float(dy), float(dz)])


def table_object():
    co = CollisionObject(id='table', operation=CollisionObject.ADD)
    co.header.frame_id = 'world'
    # top face slightly below z=0 so the robot base (resting on it) is not 'in collision'
    co.primitives = [_box(TABLE[2], TABLE[3], 0.05)]
    co.primitive_poses = [_pose(TABLE[0], TABLE[1], -0.025 - TABLE_TOP_GAP)]
    return co


def object_to_collision(o, pad=0.0):
    """pad: safety margin (m) added around parts/obstacles; receptacles are kept exact so the hand can enter."""
    co = CollisionObject(id=o.id, operation=CollisionObject.ADD)
    co.header.frame_id = 'world'
    sx, sy, sz = o.size.x, o.size.y, o.size.z
    zc = o.pose.position.z
    if o.category != 'receptacle' and pad > 0:
        if o.category == 'obstacle':
            pad *= 2            # things we must not touch (cups, tools) get a wider berth
        sx, sy, sz, zc = sx + 2 * pad, sy + 2 * pad, sz + pad, zc + pad / 2
    co.pose = _pose(o.pose.position.x, o.pose.position.y, zc, o.pose.orientation)
    if o.shape == 'bin':      # floor + 4 walls, so the hand can reach inside
        parts = [((0, 0, -sz / 2 + WALL / 2), (sx, sy, WALL)),
                 ((-sx / 2 + WALL / 2, 0, 0), (WALL, sy, sz)), ((sx / 2 - WALL / 2, 0, 0), (WALL, sy, sz)),
                 ((0, -sy / 2 + WALL / 2, 0), (sx, WALL, sz)), ((0, sy / 2 - WALL / 2, 0), (sx, WALL, sz))]
        co.primitives = [_box(*d) for _, d in parts]
        co.primitive_poses = [_pose(*c) for c, _ in parts]
    elif o.shape == 'cylinder':
        co.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[float(sz), float(max(sx, sy) / 2)])]
        co.primitive_poses = [_pose(0, 0, 0)]
    else:
        co.primitives = [_box(sx, sy, sz)]
        co.primitive_poses = [_pose(0, 0, 0)]
    return co


def scene_diff(objects, exclude=(), previous_ids=(), pad=0.01):
    """PlanningScene diff: add/update every object (minus `exclude`), remove ids that disappeared."""
    ps = PlanningScene(is_diff=True)
    ps.robot_state.is_diff = True
    current = [o for o in objects if o.id not in exclude]
    ps.world.collision_objects = [table_object()] + [object_to_collision(o, pad) for o in current]
    keep = {o.id for o in current} | {'table'}
    for gone in set(previous_ids) - keep:
        rm = CollisionObject(id=gone, operation=CollisionObject.REMOVE)
        rm.header.frame_id = 'world'
        ps.world.collision_objects.append(rm)
    return ps, keep
