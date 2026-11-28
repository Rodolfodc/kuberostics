"""The vacuum_* packages must have no dependency cycle, test dependencies included: colcon refuses to order a
workspace with one ("Unable to order packages topologically"), although every package builds alone (Art. X)."""
import pathlib
import xml.etree.ElementTree as ET

SRC = pathlib.Path(__file__).resolve().parents[2]
TAGS = ('depend', 'build_depend', 'build_export_depend', 'exec_depend', 'test_depend', 'buildtool_depend')


def graph():
    manifests = {}
    for path in SRC.glob('vacuum_*/package.xml'):
        root = ET.parse(path).getroot()
        manifests[root.findtext('name')] = {e.text.strip() for tag in TAGS for e in root.findall(tag)}
    return {name: deps & manifests.keys() for name, deps in manifests.items()}


def find_cycle(edges):
    state = {}

    def visit(node, path):
        state[node] = 1
        for nxt in sorted(edges[node]):
            if state.get(nxt) == 1:
                return path + [node, nxt]
            if nxt not in state:
                found = visit(nxt, path + [node])
                if found:
                    return found
        state[node] = 2
        return None
    for node in sorted(edges):
        if node not in state:
            found = visit(node, [])
            if found:
                return found
    return None


def test_001_R01_workspace_packages_have_no_dependency_cycle():
    assert len(graph()) >= 5
    assert find_cycle(graph()) is None, f'dependency cycle: {find_cycle(graph())}'
