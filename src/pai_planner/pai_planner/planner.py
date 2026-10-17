"""Instruction + scene graph -> validated skill plan.

A plan is a dict:
    {"status": "ok" | "clarify" | "impossible", "reasoning": str, "question": str,
     "steps": [{"skill": "pick" | "place" | "move_to", "object_id": str, "target_id": str,
                "named_target": str, "why": str}, ...]}

ClaudePlanner understands free-form language; RulePlanner is the offline fallback (and the
ablation in the benchmark). Every plan from either goes through validate() before execution.
Pure Python (no ROS) so it is unit-testable.
"""
import json
import re

SKILLS = ('pick', 'place', 'move_to')
NAMED_TARGETS = ('ready', 'camera_clear')

PLAN_SCHEMA = {
    'type': 'object',
    'properties': {
        'status': {'type': 'string', 'enum': ['ok', 'clarify', 'impossible']},
        'reasoning': {'type': 'string', 'description': 'one or two sentences'},
        'question': {'type': 'string', 'description': 'clarifying question when status is clarify, else ""'},
        'steps': {'type': 'array', 'items': {
            'type': 'object',
            'properties': {
                'skill': {'type': 'string', 'enum': list(SKILLS)},
                'object_id': {'type': 'string'},
                'target_id': {'type': 'string'},
                'named_target': {'type': 'string'},
                'why': {'type': 'string'},
            },
            'required': ['skill', 'object_id', 'target_id', 'named_target', 'why'],
            'additionalProperties': False}},
    },
    'required': ['status', 'reasoning', 'question', 'steps'],
    'additionalProperties': False,
}

SYSTEM_PROMPT = """You are the task planner of a robotic workcell: a Franka Panda arm with a parallel \
gripper that grasps parts from above. You turn an operator's instruction into a short sequence of \
skills, using ONLY the objects in the scene graph you are given (measured by the robot's camera).

Skills:
- pick(object_id): grasp one part. The gripper must be empty. Only objects with category "part" \
and graspable=true can be picked.
- place(target_id): put the part currently held into/onto a receptacle (category "receptacle"). \
The robot chooses a free spot inside it.
- move_to(named_target): "ready" or "camera_clear". Rarely needed; the executor handles this.

Rules:
- One part at a time: every pick is followed by a place.
- Never pick obstacles (cups, tools...) or receptacles. Do not move anything the instruction \
does not require. Parts that already satisfy the instruction need no steps (see relations).
- Map everyday words to the scene: "bolt", "peg", "pin" and "nut" are usually cylinders; "cube", \
"block", "brick" are boxes; "container", "basket", "box" (as a destination) may be a bin; "kit" \
usually means the tray. Colours may be described loosely ("crimson" = red).
- If the instruction refers to something that is not in the scene, or is ambiguous in a way that \
could make the robot do the wrong thing (e.g. "the red part" when there are two red parts), \
return status "clarify" with one short question and no steps.
- If it cannot be done (e.g. the part is not graspable, or no matching receptacle exists), return \
"impossible" and explain briefly.
- If held_object is not empty, the robot is already holding that part: start with a place for it \
(or put it back where it belongs if it is not part of the task).
- When history is present you are re-planning after a failure or a change in the scene: plan \
ONLY the remaining work from the current scene. Things may have moved.
- Coordinates are metres in the robot base frame: +x away from the robot, +y to the robot's left.
- Keep "why" to a few words."""


def scene_index(scene):
    return {o['id']: o for o in scene['objects']}


def validate(plan, scene):
    """Symbolically execute the plan. Returns a list of problems (empty = valid)."""
    if plan.get('status') != 'ok':
        return []
    objs = scene_index(scene)
    held = scene.get('held_object') or ''
    errors = []
    if not plan.get('steps'):
        errors.append('status is ok but there are no steps')
    for i, st in enumerate(plan.get('steps', []), 1):
        sk = st.get('skill')
        if sk not in SKILLS:
            errors.append(f'step {i}: unknown skill "{sk}"')
        elif sk == 'pick':
            o = objs.get(st.get('object_id'))
            if held:
                errors.append(f'step {i}: pick while already holding {held}')
            if o is None:
                errors.append(f'step {i}: unknown object "{st.get("object_id")}"')
            elif o['category'] != 'part':
                errors.append(f'step {i}: {o["id"]} is a {o["category"]}, not a part')
            elif not o.get('graspable', False):
                errors.append(f'step {i}: {o["id"]} is not graspable')
            else:
                held = o['id']
        elif sk == 'place':
            t = objs.get(st.get('target_id'))
            if not held:
                errors.append(f'step {i}: place with an empty gripper')
            if t is None:
                errors.append(f'step {i}: unknown target "{st.get("target_id")}"')
            elif t['category'] != 'receptacle':
                errors.append(f'step {i}: {t["id"]} is not a receptacle')
            held = ''
        elif sk == 'move_to' and st.get('named_target') not in NAMED_TARGETS:
            errors.append(f'step {i}: unknown named target "{st.get("named_target")}"')
    if held and not errors:
        errors.append(f'plan ends while still holding {held}')
    return errors


def step(skill, object_id='', target_id='', named_target='', why=''):
    return {'skill': skill, 'object_id': object_id, 'target_id': target_id, 'named_target': named_target, 'why': why}


# ---------------------------------------------------------------------------------------- Claude
class ClaudePlanner:
    name = 'claude'

    def __init__(self, client, effort='medium'):
        self.client = client
        self.effort = effort
        self.last_error = ''

    @property
    def available(self):
        return self.client is not None and self.client.available

    def plan(self, instruction, scene, history=()):
        payload = {'instruction': instruction, 'scene': scene, 'history': list(history)}
        content = [{'type': 'text', 'text': json.dumps(payload, indent=1)}]
        plan, err = self.client.json_call(SYSTEM_PROMPT, content, PLAN_SCHEMA, effort=self.effort)
        if plan is None:
            self.last_error = err
            return None
        problems = validate(plan, scene)
        if problems:   # one repair round with the validator's feedback
            content.append({'type': 'text', 'text':
                            'Your previous plan was rejected by the validator:\n- ' + '\n- '.join(problems)
                            + f'\nPrevious plan: {json.dumps(plan)}\nReturn a corrected plan.'})
            plan, err = self.client.json_call(SYSTEM_PROMPT, content, PLAN_SCHEMA, effort=self.effort)
            if plan is None:
                self.last_error = err
                return None
            problems = validate(plan, scene)
            if problems:
                return {'status': 'impossible', 'reasoning': 'planner produced an invalid plan: ' + '; '.join(problems),
                        'question': '', 'steps': []}
        return plan


# ----------------------------------------------------------------------------------------- Rules
SHAPE_WORDS = {
    'box': ('cube', 'cubes', 'block', 'blocks', 'brick', 'bricks', 'box', 'boxes'),
    'cylinder': ('cylinder', 'cylinders', 'bolt', 'bolts', 'peg', 'pegs', 'pin', 'pins', 'nut', 'nuts', 'rod', 'rods'),
}
ANY_PART = ('part', 'parts', 'item', 'items', 'object', 'objects', 'piece', 'pieces', 'thing', 'things',
            'everything', 'all')
RECEPTACLE_WORDS = {'bin': ('bin', 'bins', 'container', 'basket', 'box'), 'tray': ('tray', 'kit', 'pallet')}
PREPOSITIONS = r'\b(?:into|in|onto|on|to|inside)\b'


class RulePlanner:
    """Keyword planner: handles 'put/move X (and Y) in/on Z', 'sort ... by colour'. No paraphrase
    understanding; asks for clarification when unsure."""
    name = 'rule'

    def plan(self, instruction, scene, history=()):
        text = instruction.lower()
        objs = scene['objects']
        parts = [o for o in objs if o['category'] == 'part' and o['graspable']]
        recs = [o for o in objs if o['category'] == 'receptacle']
        placed = {(r.split()[0], r.split()[2]) for r in scene.get('relations', []) if r.split()[1] in ('in', 'on')}
        colours = sorted({o['color'] for o in objs})

        goals = []   # (part_id, receptacle_id)
        if 'sort' in text or re.search(r'by colou?r', text):
            for p in parts:
                match = [r for r in recs if r['color'] == p['color']]
                if len(match) == 1:
                    goals.append((p['id'], match[0]['id']))
            if not goals:
                return self._clarify('No part has a receptacle of the same colour. Where should the parts go?')
        else:
            clauses = [c for c in re.split(r';|\bthen\b|\.', text) if c.strip()]
            for clause in clauses:
                pieces = re.split(PREPOSITIONS, clause, maxsplit=1)
                if len(pieces) < 2:
                    continue
                what, where = pieces
                target = self._resolve_receptacle(where, recs, colours)
                if isinstance(target, str) and target.startswith('?'):
                    return self._clarify(target[1:])
                for ref in self._object_refs(what, colours):
                    found = self._resolve_parts(ref, parts)
                    if isinstance(found, str):
                        return self._clarify(found)
                    goals.extend((p['id'], target['id']) for p in found)
            if not goals:
                return self._clarify('I could not work out which part should go where. '
                                     'Please say e.g. "put the red cube in the blue bin".')

        held = scene.get('held_object') or ''
        steps = []
        for pid, rid in goals:
            if (pid, rid) in placed:
                continue
            if pid == held:
                steps.append(step('place', target_id=rid, why=f'already holding {pid}'))
                held = ''
                continue
            steps += [step('pick', object_id=pid, why='requested'), step('place', target_id=rid, why='requested')]
        if held:   # holding something unrelated: put it back in a receptacle first is not known -> ask
            return self._clarify(f'I am holding {held}. Where should I put it?')
        if not steps:
            return {'status': 'ok', 'reasoning': 'Everything requested is already in place.', 'question': '',
                    'steps': [step('move_to', named_target='camera_clear', why='nothing to do')]}
        return {'status': 'ok', 'reasoning': f'{len(goals)} goal(s) parsed by keyword rules.', 'question': '',
                'steps': steps}

    @staticmethod
    def _clarify(question):
        return {'status': 'clarify', 'reasoning': 'rule planner could not resolve the instruction',
                'question': question, 'steps': []}

    @staticmethod
    def _object_refs(text, colours):
        """Split 'the yellow block and the red cube' into [(colour, shape, plural), ...]."""
        refs = []
        for chunk in re.split(r',|\band\b|&', text):
            words = re.findall(r'[a-z]+', chunk)
            colour = next((w for w in words if w in colours), None)
            shape = next((s for s, ws in SHAPE_WORDS.items() if any(w in ws for w in words)), None)
            generic = any(w in ANY_PART for w in words)
            plural = any(w in ('all', 'every', 'everything') or (w.endswith('s') and (w in ANY_PART or
                         any(w in ws for ws in SHAPE_WORDS.values()))) for w in words)
            if colour or shape or generic:
                refs.append((colour, shape, plural))
        return refs

    @staticmethod
    def _resolve_parts(ref, parts):
        colour, shape, plural = ref
        found = [p for p in parts if (colour is None or p['color'] == colour) and (shape is None or p['shape'] == shape)]
        desc = ' '.join(w for w in (colour, shape or 'part') if w)
        if not found:
            return f'I cannot see a {desc}. Which part do you mean?'
        if len(found) > 1 and not plural:
            return f'There are {len(found)} {desc}s ({", ".join(p["id"] for p in found)}). Which one?'
        return found

    @staticmethod
    def _resolve_receptacle(text, recs, colours):
        words = re.findall(r'[a-z]+', text)
        colour = next((w for w in words if w in colours), None)
        kind = next((k for k, ws in RECEPTACLE_WORDS.items() if any(w in ws for w in words)), None)
        found = [r for r in recs if (colour is None or r['color'] == colour) and (kind is None or r['shape'] == kind)]
        if colour is None and kind is None:
            return '?Where should it go?'
        if len(found) != 1:
            return f'?I see {len(found)} matching receptacles for "{text.strip()}". Which one?'
        return found[0]
