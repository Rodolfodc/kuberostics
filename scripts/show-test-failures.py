#!/usr/bin/env python3
"""Prints the failed/errored test cases found in JUnit XML files, with the first lines of each message.
Usage: show-test-failures.py <build-dir> [<colcon-test-log-dir> [<tail-lines>]]
Reads build/*/pytest.xml and build/*/test_results/**/*.xml. With a colcon test log dir (log/latest_test) it also
prints, once per package with a failure, the last <tail-lines> lines of <log-dir>/<package>/stdout_stderr.log
(the captured output of the whole test run, including node logs printed by launch_testing).
Always exits 0; the pass/fail verdict comes from `colcon test-result`."""
import pathlib
import sys
import xml.etree.ElementTree as ET

MAX_LINES = 15


def main(build: pathlib.Path, logdir: pathlib.Path | None = None, tail: int = 60) -> None:
    files = sorted(set(build.glob('*/pytest.xml')) | set(build.glob('*/test_results/**/*.xml')))
    shown = 0
    failed_pkgs = []
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
                    if path.parent.name not in failed_pkgs:
                        failed_pkgs.append(path.parent.name)
                    name = f"{case.get('classname', '')}::{case.get('name', '')}"
                    print(f'FAILED [{kind}] {path.parent.name}: {name}')
                    text = (node.get('message') or '') + '\n' + (node.text or '')
                    for line in [ln for ln in text.splitlines() if ln.strip()][:MAX_LINES]:
                        print(f'    {line}')
    for pkg in failed_pkgs if logdir else []:
        log = logdir / pkg / 'stdout_stderr.log'
        if not log.is_file():
            print(f'==== no captured output at {log}')
            continue
        # -rA lists every passed test in the summary; drop those lines so the tail shows the failure
        lines = [ln for ln in log.read_text(errors='replace').splitlines() if not ln.startswith('PASSED ')]
        print(f'==== last {min(tail, len(lines))} of {len(lines)} lines of captured output: {pkg}')
        for line in lines[-tail:]:
            print(f'  | {line}')
        print(f'==== end {pkg}')
    print(f'{shown} failed test case(s) in {len(files)} result file(s)')


if __name__ == '__main__':
    argv = sys.argv
    main(pathlib.Path(argv[1] if len(argv) > 1 else 'build'),
         pathlib.Path(argv[2]) if len(argv) > 2 else None,
         int(argv[3]) if len(argv) > 3 else 60)
