#!/usr/bin/env python3
"""Generate C++ defaults from the single threshold registry (stdlib only)."""
import json
from pathlib import Path
import sys

def generate(source, destination):
    config = json.loads(Path(source).read_text())
    lines = ['#pragma once', '#include <cstddef>', '#include <cstdint>',
             'namespace wbr::orb_node::defaults {']
    for key, item in config.items():
        value = item['value']
        if not key.isidentifier() or type(value) is not int or value <= 0 or not item['rationale']:
            raise ValueError('invalid threshold registry')
        lines.append(f'inline constexpr std::int64_t {key} = {value};')
    lines.append('}')
    Path(destination).parent.mkdir(parents=True, exist_ok=True)
    Path(destination).write_text('\n'.join(lines) + '\n')

if __name__ == '__main__':
    generate(*sys.argv[1:])
