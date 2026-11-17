"""`ros2 run vacuum_mapping map_diff <committed.yaml> <new.yaml>`: divergence report of 003-R34.

Thin wrapper: reads two maps, runs `MapDiff`, prints (and optionally writes) the report. It never
modifies either map.
"""
from __future__ import annotations

import argparse
import sys
from typing import List, Optional

from .core.map_diff import MapDiff, format_report
from .map_io import read_map


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog='map_diff', description=__doc__.splitlines()[0])
    parser.add_argument('committed', help='YAML of the committed map (maps/house.yaml)')
    parser.add_argument('new', help='YAML of the regenerated map')
    parser.add_argument('-o', '--output', help='also write the report to this file (e.g. ..._report.md)')
    args = parser.parse_args(argv)

    try:
        committed, new = read_map(args.committed), read_map(args.new)
    except (OSError, KeyError, ValueError) as err:
        print('map_diff: %s' % err, file=sys.stderr)
        return 2
    text = format_report(MapDiff(log=lambda m: print(m, file=sys.stderr)).compare(committed, new))
    print(text, end='')
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
