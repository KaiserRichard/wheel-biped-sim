"""Score advisory windows; PASS means agreement with ASSUMED labels only."""
import argparse
import csv
import math
from pathlib import Path
import statistics
import sys

from tools.obstacle_decision.decide import Advisory
from .validate import SCENARIOS, load, validate


def _label(value):
    value = value.strip()
    if value in Advisory.__members__:
        return value
    try:
        return Advisory(int(value)).name
    except (ValueError, TypeError):
        raise ValueError(f'invalid advisory {value!r}') from None


def read_timeline(path):
    """Return strictly increasing (t, raw, gated) tuples; accept common headers."""
    with Path(path).open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        headers = {(h or '').strip().lower().replace(' ', '_'): h for h in reader.fieldnames or []}
        aliases = {'t': ('t', 'time', 'time_s', 't_s'),
                   'raw': ('raw_advisory', 'raw', 'advisory_raw'),
                   'gated': ('gated_advisory', 'gated', 'advisory_gated')}
        columns = {}
        for key, choices in aliases.items():
            columns[key] = next((headers[c] for c in choices if c in headers), None)
            if columns[key] is None:
                raise ValueError(f'missing {key} column; accepted: {", ".join(choices)}')
        rows = []
        for line, row in enumerate(reader, 2):
            try:
                t = float(row[columns['t']])
                if not math.isfinite(t) or t < 0 or (rows and t <= rows[-1][0]):
                    raise ValueError('timestamps must be finite, nonnegative and strictly increasing')
                rows.append((t, _label(row[columns['raw']]), _label(row[columns['gated']])))
            except (ValueError, TypeError, AttributeError) as exc:
                raise ValueError(f'{path}:{line}: {exc}') from exc
    return rows


def score_scenario(scenario, rows, *, settling_s=.5, min_agreement=.9):
    """Score raw and gated separately; require every gated window to pass.

    Agreement is sample based after the margin. Coverage is observed time,
    capped at 1.5 median frame periods per sample, preventing truncated/sparse
    timelines from passing on a handful of correct samples. First correct and
    flicker use the full window; expected boundary changes are not flicker.
    """
    validate(scenario)
    if not math.isfinite(settling_s) or settling_s < 0:
        raise ValueError('settling_s must be finite and nonnegative')
    if not math.isfinite(min_agreement) or not 0 <= min_agreement <= 1:
        raise ValueError('min_agreement must be between 0 and 1')
    previous = -1.
    for row in rows:
        if len(row) != 3 or not math.isfinite(row[0]) or row[0] < 0 or row[0] <= previous:
            raise ValueError('invalid timeline or nonincreasing timestamps')
        if any(label not in Advisory.__members__ for label in row[1:]):
            raise ValueError('invalid timeline advisory')
        previous = row[0]
    periods = [b[0]-a[0] for a, b in zip(rows, rows[1:])]
    period = statistics.median(periods) if periods else 0.
    windows = []
    for win in scenario.get('expected', []):
        start, end = win['t_from'], win['t_to']
        scored_start = min(end, start + settling_s)
        all_rows = [row for row in rows if start <= row[0] < end]
        scored = [row for row in all_rows if scored_start <= row[0]]
        coverage_s = 0.
        for i, row in enumerate(rows):
            stop = rows[i+1][0] if i+1 < len(rows) else row[0]+period
            # Credit only bounded observation intervals, never arbitrary gaps.
            stop = min(stop, row[0] + 1.5*period, end)
            coverage_s += max(0., stop-max(row[0], scored_start))
        span = end-scored_start
        coverage = min(1., coverage_s/span) if span > 0 else 0.
        result = dict(t_from=start, t_to=end, advisory=win['advisory'],
                      samples=len(scored), coverage_pct=100*coverage)
        for index, channel in ((1, 'raw'), (2, 'gated')):
            correct = sum(row[index] == win['advisory'] for row in scored)
            agreement = correct/len(scored) if scored else 0.
            first = next((row[0] for row in all_rows if row[index] == win['advisory']), None)
            flicker = sum(a[index] != b[index] for a, b in zip(all_rows, all_rows[1:]))
            result[channel] = dict(agreement_pct=100*agreement, first_correct_s=first,
                                   flicker_count=flicker,
                                   passed=bool(scored) and agreement >= min_agreement and coverage >= min_agreement)
        windows.append(result)
    return dict(name=scenario['name'], windows=windows,
                passed=bool(windows) and all(w['gated']['passed'] for w in windows))


def score_all(results_dir, scenarios_dir=SCENARIOS, **options):
    """Match NAME/timeline.csv or NAME.csv, fail missing/invalid results."""
    results_dir, scenarios_dir = Path(results_dir), Path(scenarios_dir)
    output = []
    paths = sorted(scenarios_dir.glob('*.json'))
    if not paths:
        raise ValueError(f'no scenarios in {scenarios_dir}')
    names = set()
    for path in paths:
        try:
            scenario = load(path)
            name = scenario['name']
            if Path(name).name != name or name in ('.', '..'):
                raise ValueError('scenario name must be a filename component for score_all')
            if name in names:
                raise ValueError(f'duplicate scenario name: {name}')
            names.add(name)
            candidates = (results_dir/name/'timeline.csv', results_dir/f'{name}.csv')
            matches = [p for p in candidates if p.is_file()]
            if len(matches) != 1:
                raise ValueError(f'expected one timeline for {name}, found {len(matches)}')
            output.append(score_scenario(scenario, read_timeline(matches[0]), **options))
        except (OSError, ValueError, TypeError) as exc:
            output.append(dict(name=path.stem, windows=[], passed=False, error=str(exc)))
    return output


def markdown(results):
    lines = ['| Scenario / window (s) | Expected | Raw agree | Gated agree | Coverage | First correct raw / gated (s) | Flicker raw / gated | Result |',
             '|---|---|---|---|---|---|---|---|']
    def number(value):
        return 'never' if value is None else f'{value:.3f}'
    for result in results:
        name = result['name'].replace('|', '/').replace('\n', ' ')
        for win in result['windows']:
            raw, gated = win['raw'], win['gated']
            lines.append(f'| {name} [{win["t_from"]:g}, {win["t_to"]:g}) | {win["advisory"]} | '
                         f'{raw["agreement_pct"]:.1f}% | {gated["agreement_pct"]:.1f}% | '
                         f'{win["coverage_pct"]:.1f}% | {number(raw["first_correct_s"])} / {number(gated["first_correct_s"])} | '
                         f'{raw["flicker_count"]} / {gated["flicker_count"]} | {"PASS" if gated["passed"] else "FAIL"} |')
        verdict = "NOT SCORED (no expected windows)" if not result["windows"] and not result.get("error") else ("PASS" if result["passed"] else "FAIL")
        lines.append(f'| {name} total | — | — | — | — | — | — | {verdict} |')
        if result.get('error'):
            lines.append('')
            lines.append(f'{name}: {result["error"]}')
            lines.append('')
    return '\n'.join(lines)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    batch = bool(argv and argv[0] == 'score_all')
    if batch:
        argv.pop(0)
    parser = argparse.ArgumentParser(description=__doc__)
    if batch:
        parser.add_argument('results_dir', type=Path)
        parser.add_argument('--scenarios-dir', type=Path, default=SCENARIOS)
    else:
        parser.add_argument('scenario', type=Path)
        parser.add_argument('timeline', type=Path)
    parser.add_argument('--settling-s', type=float, default=.5)
    parser.add_argument('--min-agreement', type=float, default=.9)
    args = parser.parse_args(argv)
    options = dict(settling_s=args.settling_s, min_agreement=args.min_agreement)
    try:
        if batch:
            results = score_all(args.results_dir, args.scenarios_dir, **options)
        else:
            results = [score_scenario(load(args.scenario), read_timeline(args.timeline), **options)]
    except (OSError, ValueError, TypeError) as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        return 1
    print(markdown(results))
    return int(not all(result['passed'] for result in results))


if __name__ == '__main__':
    raise SystemExit(main())
