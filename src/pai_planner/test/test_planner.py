"""Offline tests: rule planner parsing, validator, and the Claude planner's repair loop (mocked)."""
from pai_planner.planner import ClaudePlanner, RulePlanner, validate


def obj(i, color, shape, category, graspable=True):
    return {'id': i, 'label': i.replace('_', ' '), 'color': color, 'shape': shape, 'category': category,
            'position_m': [0.4, 0.0, 0.02], 'yaw_deg': 0, 'size_cm': [4, 4, 4], 'graspable': graspable}


SCENE = {
    'objects': [obj('blue_bin', 'blue', 'bin', 'receptacle', False), obj('green_bin', 'green', 'bin', 'receptacle', False),
                obj('gray_tray', 'gray', 'tray', 'receptacle', False),
                obj('red_cube', 'red', 'box', 'part'), obj('blue_cylinder', 'blue', 'cylinder', 'part'),
                obj('yellow_block', 'yellow', 'box', 'part'), obj('green_cylinder', 'green', 'cylinder', 'part'),
                obj('white_cylinder', 'white', 'cylinder', 'obstacle', False)],
    'relations': ['red_cube on table', 'blue_cylinder on table', 'yellow_block on table', 'green_cylinder on table'],
    'held_object': '',
}


def pairs(plan):
    s = plan['steps']
    return [(s[i]['object_id'], s[i + 1]['target_id']) for i in range(0, len(s), 2)]


def test_simple_put():
    p = RulePlanner().plan('Put the red cube in the blue bin.', SCENE)
    assert p['status'] == 'ok' and pairs(p) == [('red_cube', 'blue_bin')]
    assert validate(p, SCENE) == []


def test_bolt_synonym_and_kit():
    p = RulePlanner().plan('place the yellow block and the blue bolt on the kit tray', SCENE)
    assert pairs(p) == [('yellow_block', 'gray_tray'), ('blue_cylinder', 'gray_tray')]


def test_sort_by_colour():
    p = RulePlanner().plan('Sort the parts by colour', SCENE)
    assert sorted(pairs(p)) == [('blue_cylinder', 'blue_bin'), ('green_cylinder', 'green_bin')]


def test_ambiguous_asks():
    p = RulePlanner().plan('put the cylinder in the blue bin', SCENE)
    assert p['status'] == 'clarify' and 'Which one' in p['question']


def test_missing_object_asks():
    p = RulePlanner().plan('put the purple cylinder in the green bin', SCENE)
    assert p['status'] == 'clarify'


def test_already_done_is_noop():
    scene = dict(SCENE, relations=['red_cube in blue_bin'])
    p = RulePlanner().plan('put the red cube in the blue bin', scene)
    assert [s['skill'] for s in p['steps']] == ['move_to']


def test_validator_catches_bad_plans():
    bad = {'status': 'ok', 'steps': [
        {'skill': 'pick', 'object_id': 'white_cylinder', 'target_id': '', 'named_target': '', 'why': ''},
        {'skill': 'place', 'object_id': '', 'target_id': 'red_cube', 'named_target': '', 'why': ''}]}
    errs = validate(bad, SCENE)
    assert any('obstacle' in e for e in errs) and any('not a receptacle' in e for e in errs)


class FakeClient:
    """Returns an invalid plan first, then a valid one: exercises the repair round."""
    available = True

    def __init__(self):
        self.calls = 0

    def json_call(self, system, content, schema, effort='medium'):
        self.calls += 1
        if self.calls == 1:
            return {'status': 'ok', 'reasoning': '', 'question': '', 'steps': [
                {'skill': 'pick', 'object_id': 'crimson_cube', 'target_id': '', 'named_target': '', 'why': ''},
                {'skill': 'place', 'object_id': '', 'target_id': 'blue_bin', 'named_target': '', 'why': ''}]}, ''
        assert 'rejected by the validator' in content[-1]['text']
        return {'status': 'ok', 'reasoning': '', 'question': '', 'steps': [
            {'skill': 'pick', 'object_id': 'red_cube', 'target_id': '', 'named_target': '', 'why': ''},
            {'skill': 'place', 'object_id': '', 'target_id': 'blue_bin', 'named_target': '', 'why': ''}]}, ''


def test_claude_repair_round():
    c = FakeClient()
    p = ClaudePlanner(c).plan('drop the crimson block in the blue container', SCENE)
    assert c.calls == 2 and pairs(p) == [('red_cube', 'blue_bin')]
