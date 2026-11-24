#!/usr/bin/env python3
"""Prints the failed/errored test cases found in JUnit XML files, with the first lines of each message.
Usage: show-test-failures.py <build-dir>   (reads build/*/pytest.xml and build/*/test_results/**/*.xml)
Always exits 0; the pass/fail verdict comes from `colcon test-result`."""
import pathlib
import sys
import xml.etree.ElementTree as ET

MAX_LINES = 15


def main(build: pathlib.Path) -> None:
    files = sorted(set(build.glob('*/pytest.xml')) | set(build.glob('*/test_results/**/*.xml')))
    shown = 0
    for path in files:
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as exc:
            print(f'!! unparsable {path}: {exc}')
            continue
        for case in root.iter('testcase'):
            for kind in ('failure', 'error'):
                for node in case.findall(kind):
                    shown += 1
                    name = f"{case.get('classname', '')}::{case.get('name', '')}"
                    print(f'FAILED [{kind}] {path.parent.name}: {name}')
                    text = (node.get('message') or '') + '\n' + (node.text or '')
                    for line in [ln for ln in text.splitlines() if ln.strip()][:MAX_LINES]:
                        print(f'    {line}')
    print(f'{shown} failed test case(s) in {len(files)} result file(s)')


if __name__ == '__main__':
    main(pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else 'build'))
