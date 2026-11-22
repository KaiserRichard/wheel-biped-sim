"""Schema checks and hand-calculated scorer examples; no simulator dependency."""
import contextlib
import copy
import io
import json
from pathlib import Path
import tempfile
import unittest

from tools.sim_scenarios.score import main, markdown, read_timeline, score_all, score_scenario
from tools.sim_scenarios.validate import SCENARIOS, load, validate


class LibraryTests(unittest.TestCase):
    def test_all_scenarios_validate(self):
        paths = sorted(SCENARIOS.glob('*.json'))
        self.assertEqual(len(paths), 13)
        names = []
        for path in paths:
            with self.subTest(path=path.name):
                data = load(path)
                self.assertEqual(data['status'], 'ASSUMED scene')
                self.assertTrue(data["expected"] or data["name"] == "at_min_height_020")
                names.append(data['name'])
        self.assertEqual(len(names), len(set(names)))

    def test_invalid_schema_and_sanity(self):
        original = load(SCENARIOS/'lab_box132.json')
        changes = [lambda d: d.update(extra=1),
                   lambda d: d.update(robot=[]),
                   lambda d: d['robot'].update(speed_m_s=True),
                   lambda d: d['robot'].update(l0=float('nan')),
                   lambda d: d['robot'].update(jump_at_s=6),
                   lambda d: d['obstacles'][0].update(x_face=-1),
                   lambda d: d['obstacles'][0].update(width=0),
                   lambda d: d['obstacles'][0].update(type='sphere'),
                   lambda d: d['obstacles'][0].update(foo=1),
                   lambda d: d['expected'][0].update(t_to=7),
                   lambda d: d['expected'][0].update(advisory='JUMP'),
                   lambda d: d['expected'].append(d['expected'][0]),
                   lambda d: d.update(status='MEASURED')]
        for change in changes:
            data = copy.deepcopy(original)
            change(data)
            with self.subTest(data=data), self.assertRaises(ValueError):
                validate(data)

    def test_expected_optional(self):
        data = load(SCENARIOS/'lab_empty.json')
        del data['expected']
        validate(data)
        self.assertFalse(score_scenario(data, [])['passed'])


class ScorerTests(unittest.TestCase):
    def setUp(self):
        self.scene = load(SCENARIOS/'lab_empty.json')
        self.scene['robot']['duration_s'] = 2.
        self.scene['expected'][0]['t_to'] = 2.
        self.rows = [(i/10, 'NONE', 'NONE') for i in range(20)]

    def result(self, rows):
        return score_scenario(self.scene, rows)

    def test_perfect(self):
        result = self.result(self.rows)
        win = result['windows'][0]
        self.assertTrue(result['passed'])
        self.assertEqual(win['samples'], 15)
        self.assertEqual(win['gated']['agreement_pct'], 100.)
        self.assertAlmostEqual(win['coverage_pct'], 100.)
        self.assertEqual(win['gated']['first_correct_s'], 0.)
        self.assertEqual(win['gated']['flicker_count'], 0)
        self.assertIn('PASS', markdown([result]))

    def test_flickery(self):
        rows = [(t, r, 'UNCERTAIN' if i in (6, 8) else g)
                for i, (t, r, g) in enumerate(self.rows)]
        win = self.result(rows)['windows'][0]
        self.assertAlmostEqual(win['gated']['agreement_pct'], 100*13/15)
        self.assertEqual(win['gated']['flicker_count'], 4)
        self.assertEqual(win['raw']['agreement_pct'], 100.)
        self.assertFalse(win['gated']['passed'])

    def test_late(self):
        rows = [(t, r, 'UNCERTAIN' if t < 1 else g) for t, r, g in self.rows]
        result = self.result(rows)
        self.assertFalse(result['passed'])
        win = result['windows'][0]
        self.assertAlmostEqual(win['gated']['agreement_pct'], 100*10/15)
        self.assertEqual(win['gated']['first_correct_s'], 1.)
        self.assertEqual(win['gated']['flicker_count'], 1)

    def test_wrong(self):
        result = self.result([(t, 'STOP_CANDIDATE', 'STOP_CANDIDATE') for t, _, _ in self.rows])
        win = result['windows'][0]['gated']
        self.assertFalse(result['passed'])
        self.assertEqual(win['agreement_pct'], 0.)
        self.assertIsNone(win['first_correct_s'])
        self.assertEqual(win['flicker_count'], 0)

    def test_settling_and_tolerance(self):
        rows = [(t, r, 'UNCERTAIN' if t < .5 or t == 1 else g) for t, r, g in self.rows]
        result = self.result(rows)
        self.assertTrue(result['passed'])
        self.assertAlmostEqual(result['windows'][0]['gated']['agreement_pct'], 100*14/15)
        self.assertEqual(result['windows'][0]['gated']['first_correct_s'], .5)

    def test_each_window_must_pass_and_boundary_is_not_flicker(self):
        self.scene['expected'] = [dict(t_from=0., t_to=1., advisory='NONE', note='first'),
                                  dict(t_from=1., t_to=2., advisory='UNCERTAIN', note='second')]
        rows = [(t, r, 'UNCERTAIN' if t >= 1 else g) for t, r, g in self.rows]
        result = self.result(rows)
        self.assertTrue(result['passed'])
        self.assertEqual([w['gated']['flicker_count'] for w in result['windows']], [0, 0])
        self.assertEqual(result['windows'][1]['gated']['first_correct_s'], 1.)
        self.assertFalse(self.result(self.rows)['passed'])

    def test_empty_truncated_and_gappy_fail(self):
        for rows in ([], self.rows[:8], self.rows[5:6], self.rows[:6]+self.rows[15:]):
            with self.subTest(rows=rows):
                self.assertFalse(self.result(rows)['passed'])

    def test_short_window_cannot_silently_pass(self):
        self.scene['expected'][0]['t_to'] = .3
        self.assertFalse(self.result(self.rows)['passed'])

    def test_bad_parameters(self):
        for kwargs in (dict(settling_s=-1), dict(settling_s=float('nan')),
                       dict(min_agreement=1.1), dict(min_agreement=float('nan'))):
            with self.assertRaises(ValueError):
                score_scenario(self.scene, self.rows, **kwargs)

    def test_csv_variants_and_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scenario = root/'scene.json'
            scenario.write_text(json.dumps(self.scene))
            timeline = root/'timeline.csv'
            for header in ('t,raw_advisory,gated_advisory', 'time_s,raw advisory,gated advisory', 't,raw,gated'):
                timeline.write_text(header+'\n'+''.join(f'{t},0,NONE\n' for t, _, _ in self.rows))
                self.assertEqual(read_timeline(timeline), self.rows)
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(main([str(scenario), str(timeline)]), 0)
            timeline.write_text('t,raw,gated\n0,NONE,JUMP_CANDIDATE\n')
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main([str(scenario), str(timeline)]), 1)

    def test_bad_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'timeline.csv'
            for text in ('t,raw\n0,NONE\n', 't,raw,gated\n0,NONE,BOGUS\n',
                         't,raw,gated\nnan,NONE,NONE\n', 't,raw,gated\n-1,NONE,NONE\n',
                         't,raw,gated\n0,NONE,NONE\n0,NONE,NONE\n',
                         't,raw,gated\n1,NONE,NONE\n0,NONE,NONE\n'):
                path.write_text(text)
                with self.subTest(text=text), self.assertRaises(ValueError):
                    read_timeline(path)

    def test_score_all_missing_nested_flat_and_ambiguous(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scenarios = root/'scenarios'
            scenarios.mkdir()
            (scenarios/'scene.json').write_text(json.dumps(self.scene))
            results = root/'results'
            results.mkdir()
            missing = score_all(results, scenarios)
            self.assertFalse(missing[0]['passed'])
            self.assertIn('found 0', missing[0]['error'])
            csv_text = 't,raw,gated\n'+''.join(f'{t},NONE,NONE\n' for t, _, _ in self.rows)
            (results/'lab_empty.csv').write_text(csv_text)
            self.assertTrue(score_all(results, scenarios)[0]['passed'])
            nested = results/'lab_empty'
            nested.mkdir()
            (nested/'timeline.csv').write_text(csv_text)
            self.assertFalse(score_all(results, scenarios)[0]['passed'])
            (results/'lab_empty.csv').rename(results/'unused.csv')
            self.assertTrue(score_all(results, scenarios)[0]['passed'])
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(['score_all', str(results), '--scenarios-dir', str(scenarios)]), 0)


if __name__ == '__main__':
    unittest.main()
