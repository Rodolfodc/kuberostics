"""A YAML loader that rejects duplicate keys, applied to the contract and to this package's configs.

PyYAML silently keeps the last of two equal keys; a bad merge once left two `note:` keys in one topics.yaml entry
and dropped another entry's note. This test makes that fail loudly (Art. I, contract first).
"""
import pathlib

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parent.parent


class UniqueKeyLoader(yaml.SafeLoader):
    def construct_mapping(self, node, deep=False):
        seen = set()
        for key_node, _ in node.value:
            key = self.construct_object(key_node, deep=deep)
            if key in seen:
                raise yaml.constructor.ConstructorError(
                    None, None, f'duplicate key {key!r}', key_node.start_mark)
            seen.add(key)
        return super().construct_mapping(node, deep)


def load_unique(path):
    return yaml.load(pathlib.Path(path).read_text(), Loader=UniqueKeyLoader)


def contract_path():
    for parent in PKG.parents:
        candidate = parent / 'specs' / '000-system' / 'contracts' / 'topics.yaml'
        if candidate.exists():
            return candidate
    return None


def test_the_loader_rejects_duplicate_keys():
    with pytest.raises(yaml.YAMLError):
        yaml.load('a: 1\nnote: x\nnote: y\n', Loader=UniqueKeyLoader)


def test_005_R35_contract_topics_yaml_has_no_duplicate_keys():
    path = contract_path()
    if path is None:
        pytest.skip('specs/ is not in this tree')
    load_unique(path)


@pytest.mark.parametrize('path', sorted((PKG / 'config').glob('*.yaml')), ids=lambda p: p.name)
def test_005_R35_package_configs_have_no_duplicate_keys(path):
    load_unique(path)
