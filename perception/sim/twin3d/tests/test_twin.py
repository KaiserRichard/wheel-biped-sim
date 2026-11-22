import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import mujoco
import numpy as np

from sim.twin3d.model import Plant
from sim.twin3d.control import Jump, design, Controller
from sim.twin3d.check import evaluate, aggregate


class ModelTests(unittest.TestCase):
    def test_compile_joints_actuators_contact_gravity(self):
        plant = Plant(1.0)
        self.assertEqual(plant.model.nq, 9)
        self.assertEqual(plant.model.nu, 6)
        self.assertEqual(plant.model.nmocap, 0)
        self.assertEqual(plant.model.neq, 0)
        np.testing.assert_allclose(plant.model.opt.gravity, (0, 0, -9.81))
        for side in ('left', 'right'):
            for name in ('hip_', 'length_', 'wheel_'):
                self.assertIn(name+side, plant.q)
                self.assertIn(name+side, plant.u)
            np.testing.assert_allclose(plant.model.joint('length_'+side).range, (.06, .21))
            self.assertGreater(plant.model.geom('wheel_'+side).contype[0], 0)
        data = plant.data()
        for _ in range(5):
            mujoco.mj_step(plant.model, data)
        self.assertTrue(plant.contacts(data)['left'])

    def test_true_ballistic_flight_without_actuation(self):
        plant = Plant()
        data = plant.data()
        data.qpos[plant.q['base_z']] += 1.0
        mujoco.mj_forward(plant.model, data)
        self.assertFalse(plant.contacts(data)['left'])
        # ASSUMED 1e-5 joint armature slightly reduces free-fall acceleration.
        self.assertAlmostEqual(data.qacc[plant.q['base_z']], -9.81, delta=5e-5)
        self.assertTrue(np.allclose(data.qfrc_applied, 0))


class ControlTests(unittest.TestCase):
    def test_lqr_shape_and_stability_three_lengths(self):
        for length in (.08, .12, .20):
            with self.subTest(length=length):
                gain, record = design(length)
                self.assertEqual(gain.shape, (2, 6))
                A = np.array(record['A_discrete_actual'])
                B = np.array(record['B_discrete_actual'])
                self.assertLess(max(abs(np.linalg.eigvals(A-B@gain))), 1)
                self.assertGreater(max(abs(np.linalg.eigvals(A))), 1)

    def test_source_phase_thresholds_strict_and_fixed_targets(self):
        jump = Jump()
        self.assertEqual(jump.update(.12, True), .08)
        for _ in range(9):
            jump.update(.099)
        self.assertEqual(jump.phase, 'CROUCH')
        self.assertEqual(jump.update(.099), .21)
        for _ in range(9):
            jump.update(.19)
        self.assertEqual(jump.phase, 'THRUST')
        self.assertEqual(jump.update(.19), .12)
        for _ in range(9):
            jump.update(.11)
        self.assertEqual(jump.phase, 'RETRACT')
        self.assertEqual(jump.update(.11), .12)
        self.assertEqual(jump.phase, 'SETTLE')
        jump.update(.12, True)
        self.assertEqual(jump.phase, 'SETTLE')

    def test_jump_counts_accumulate_and_require_both_sides(self):
        jump = Jump()
        jump.update(.12, True)
        for left, right in ((.09, .11), (.11, .09), (.09, .09)):
            jump.update({'left': left, 'right': right})
        self.assertEqual(jump.passes, {'left': 2, 'right': 2})
        for _ in range(7):
            jump.update({'left': .09, 'right': .09})
        self.assertEqual(jump.phase, 'CROUCH')
        jump.update({'left': .09, 'right': .09})
        self.assertEqual(jump.phase, 'THRUST')
        self.assertEqual(jump.passes, {'left': 0, 'right': 0})

    def test_controller_never_repositions_the_robot(self):
        plant = Plant()
        data = plant.data(.01)
        before = data.qpos.copy()
        Controller().update(plant, data, .3, True)
        np.testing.assert_array_equal(data.qpos, before)
        self.assertTrue(np.all(abs(data.ctrl) <= plant.model.actuator_ctrlrange[:, 1]))
        data.qpos[plant.q['base_z']] += 1.0
        mujoco.mj_forward(plant.model, data)
        Controller().update(plant, data, .3, True)
        for side in ('left', 'right'):
            self.assertEqual(data.ctrl[plant.u['wheel_'+side]], 0)


def fake(scenario):
    duration = 30 if scenario == 'stand' else 6
    rows = []
    for i in range(duration*100+1):
        t = i/100
        x = t*.3 if scenario == 'drive' else t*.5 if scenario == 'jump_box' else 0
        air = scenario == 'jump_box' and 1.5 <= t < 2.8
        phase = ('CROUCH' if .5 <= t < .6 else 'THRUST' if .6 <= t < .7 else
                 'RETRACT' if .7 <= t < .8 else 'SETTLE' if t >= .8 else 'STAND')
        row = dict(t=t, scenario=scenario, seed=0, x=x, pitch=0, vx=.3 if scenario == 'drive' else 0,
                   base_ground=0, bad_box=0, wheel_box=0, contact_left=int(not air),
                   contact_right=int(not air), flight=int(air), phase=phase,
                   box_face=1, box_depth=.12, box_height=.08,
                   raw_advisory='', gated_advisory='', gated_confidence='')
        for side in ('left', 'right'):
            row['wheel_'+side+'_x'] = x
            row['wheel_'+side+'_bottom'] = .15 if air else 0
        rows.append(row)
    return rows


class CheckTests(unittest.TestCase):
    def test_positive_evidence_and_seed_aggregation(self):
        results = [evaluate(fake(s)) for s in ('stand', 'drive')]
        for seed in range(10):
            rows = fake('jump_box')
            for r in rows:
                r['seed'] = seed
            results.append(evaluate(rows))
        self.assertTrue(aggregate(results)['passed'])
        results[-1]['passed'] = False
        results[-2]['passed'] = False
        self.assertTrue(aggregate(results)['passed'])
        results[-3]['passed'] = False
        self.assertFalse(aggregate(results)['passed'])
        self.assertFalse(aggregate(results[:3])['passed'])

    def test_rejects_fall_ground_short_run_and_missing_time(self):
        for key, value in (('pitch', math.radians(30)), ('base_ground', 1), ('pitch', float('nan'))):
            rows = fake('stand')
            rows[50][key] = value
            self.assertFalse(evaluate(rows)['passed'])
        self.assertFalse(evaluate(fake('stand')[:-1])['passed'])
        rows = fake('stand')
        rows.pop(30)
        self.assertFalse(evaluate(rows)['passed'])
        self.assertFalse(evaluate([])['passed'])
        rows = fake('stand')
        rows[50]['physics_warning'] = 1
        self.assertFalse(evaluate(rows)['passed'])

    def test_both_wheels_must_clear_and_land(self):
        for key, value in (('bad_box', 1), ('wheel_box', 1), ('wheel_right_bottom', .079)):
            rows = fake('jump_box')
            rows[210][key] = value
            self.assertFalse(evaluate(rows)['passed'])
        self.assertFalse(evaluate(fake('jump_box')[:350])['passed'])

    def test_gate_must_cover_both_active_phases_and_inhibit(self):
        rows = fake('jump_box')
        for r in rows:
            r.update(raw_advisory='JUMP_CANDIDATE', gated_advisory='UNCERTAIN', gated_confidence=0)
        self.assertTrue(evaluate(rows)['gate_passed'])
        rows[55]['gated_confidence'] = .1
        self.assertFalse(evaluate(rows)['passed'])
        rows[55]['gated_confidence'] = 0
        for r in rows:
            if r['phase'] == 'THRUST':
                r['raw_advisory'] = ''
        self.assertFalse(evaluate(rows)['gate_passed'])
        rows = fake('jump_box')
        for r in rows:
            r['perception'] = 1
        self.assertFalse(evaluate(rows)['passed'])


class PerceptionTests(unittest.TestCase):
    def test_dynamic_telemetry_inhibits_active_phases_with_render_stub(self):
        from sim.twin3d.run import Perception
        from tools.obstacle_decision.decide import Advisory
        plant = Plant()
        data = plant.data()
        renderer = Mock()
        renderer.render.return_value = np.full((480, 640), .5, dtype=np.float32)
        loop = Perception(plant, renderer)
        frame = SimpleNamespace(plane_valid=True)
        decision = SimpleNamespace(advisory=Advisory.JUMP_CANDIDATE, confidence=.9)
        with patch('tools.simcam.run.update_camera'), \
                patch('tools.obstacle_decision.detect.process_frame', return_value=(frame, {})), \
                patch('tools.obstacle_decision.decide.update', return_value=(decision, {})):
            for t, phase in ((0, 'STAND'), (.1, 'CROUCH'), (.2, 'THRUST')):
                data.time = t
                row = loop.update(plant, data, phase, dict(left=True, right=True))
                if phase != 'STAND':
                    self.assertEqual(row['raw_advisory'], 'JUMP_CANDIDATE')
                    self.assertEqual(row['gated_advisory'], 'UNCERTAIN')
                    self.assertEqual(row['gated_confidence'], 0)
            # Holding a frame does not rewrite acquisition time or call render.
            data.time = .201
            loop.update(plant, data, 'THRUST', dict(left=True, right=True))
            self.assertEqual(loop.stamp, .2)
            self.assertEqual(renderer.render.call_count, 3)
            data.time = .3
            loop.update(plant, data, 'RETRACT', dict(left=False, right=False))
            self.assertIsNotNone(loop.result)
            data.time = .301
            loop.update(plant, data, 'RETRACT', dict(left=False, right=False))
            self.assertIsNotNone(loop.result)  # Do not repeatedly reset in flight.
            data.time = .302
            loop.update(plant, data, 'SETTLE', dict(left=True, right=True))
            self.assertIsNone(loop.result)  # First contact invalidates held history.


if __name__ == '__main__':
    unittest.main()
