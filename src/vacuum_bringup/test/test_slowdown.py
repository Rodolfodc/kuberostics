"""Slow-simulator scaling of the Nav2 wall-clock timeouts (pure helper + launch argument parsing, no ROS graph).

Requirements: 005-R34 (nav goal timeout, scaled with the same factor), 005-R19 (stock Nav2 configuration; the default
factor 1.0 leaves the files untouched). The graph test with a running Nav2 is test_nav_slowdown_graph.py.
"""
import importlib.util
import math
import pathlib

import pytest
import yaml

from vacuum_bringup.slowdown import NAV_TIMEOUT_PARAMS, parse_factor, scale_bt_xml, scale_nav_params

PKG = pathlib.Path(__file__).resolve().parent.parent


@pytest.fixture(scope='module')
def params():
    return yaml.safe_load((PKG / 'config' / 'nav2_params.yaml').read_text())


@pytest.fixture(scope='module')
def nav_launch():
    spec = importlib.util.spec_from_file_location('nav_launch_slow', PKG / 'launch' / 'nav.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('value, expected', [('1', 1.0), (1.0, 1.0), ('11.5', 11.5), (0.5, 0.5)])
def test_005_R34_parse_factor_accepts_positive_numbers(value, expected):
    assert parse_factor(value) == expected


@pytest.mark.parametrize('value', ['0', '-2', 'nan', 'inf', 'abc', '', None])
def test_005_R34_parse_factor_rejects_everything_else(value):
    with pytest.raises(ValueError):
        parse_factor(value)


def get_path(params, node, path):
    holder = params[node]['ros__parameters']
    for key in path:
        holder = holder[key]
    return holder


def test_005_R34_every_listed_timeout_exists_in_the_shipped_yaml(params):
    for node, path in NAV_TIMEOUT_PARAMS:
        assert isinstance(get_path(params, node, path), (int, float)), (node, path)


def test_005_R34_scale_multiplies_only_the_listed_timeouts(params):
    scaled = scale_nav_params(params, 4.0)
    listed = {(n, p) for n, p in NAV_TIMEOUT_PARAMS}
    for node, path in listed:
        assert get_path(scaled, node, path) == pytest.approx(4.0 * get_path(params, node, path))
    # the 15 s progress allowance of the live run becomes 60 s, the 1 s costmap timeouts 4 s
    assert get_path(scaled, 'controller_server', ('progress_checker', 'movement_time_allowance')) == 60.0
    # the rest is identical: strip the scaled keys and compare
    for doc in (params, scaled):
        for node, path in listed:
            holder = doc[node]['ros__parameters']
            for key in path[:-1]:
                holder = holder[key]
            del holder[path[-1]]
    assert scaled == params


def test_005_R34_scale_does_not_modify_its_input_and_factor_one_is_identity(params):
    before = yaml.safe_dump(params)
    scale_nav_params(params, 7.0)
    assert yaml.safe_dump(params) == before
    assert scale_nav_params(params, 1.0) == params


def test_005_R34_scale_skips_missing_keys():
    assert scale_nav_params({'controller_server': {'ros__parameters': {'use_sim_time': False}}}, 3.0) == \
        {'controller_server': {'ros__parameters': {'use_sim_time': False}}}
    assert scale_nav_params({}, 3.0) == {}


def test_005_R34_bt_timeout_is_scaled_and_the_rest_of_the_text_is_kept():
    text = (PKG / 'config' / 'nav_bt.xml').read_text()
    scaled = scale_bt_xml(text, 10.0)
    assert 'msec="1800000"' in scaled and 'msec="180000"' not in scaled
    assert scaled.replace('msec="1800000"', 'msec="180000"') == text
    assert scale_bt_xml(text, 1.0) == text


def test_005_R34_bt_scaling_handles_attribute_order_and_fractions():
    assert scale_bt_xml('<Timeout name="x" msec="2500">', 0.5) == '<Timeout name="x" msec="1250">'
    assert scale_bt_xml('<Wait msec="100"/>', 9.0) == '<Wait msec="100"/>'      # only the Timeout decorator


def test_005_R34_default_launch_uses_the_shipped_files_untouched(nav_launch, tmp_path):
    params, bt = str(PKG / 'config' / 'nav2_params.yaml'), str(PKG / 'config' / 'nav_bt.xml')
    assert nav_launch.slowed_files(params, bt, '1.0', directory=str(tmp_path)) == (params, bt)
    assert list(tmp_path.iterdir()) == []


def test_005_R34_slow_launch_writes_scaled_overlays_and_leaves_the_sources(nav_launch, tmp_path, params):
    src_params, src_bt = PKG / 'config' / 'nav2_params.yaml', PKG / 'config' / 'nav_bt.xml'
    original = src_params.read_text(), src_bt.read_text()
    out_params, out_bt = nav_launch.slowed_files(str(src_params), str(src_bt), '12', directory=str(tmp_path))
    assert pathlib.Path(out_params).parent == tmp_path and out_params != str(src_params)
    overlay = yaml.safe_load(pathlib.Path(out_params).read_text())
    assert get_path(overlay, 'controller_server', ('progress_checker', 'movement_time_allowance')) == 180.0
    assert get_path(overlay, 'planner_server', ('costmap_update_timeout',)) == 12.0
    assert 'msec="2160000"' in pathlib.Path(out_bt).read_text()
    assert (src_params.read_text(), src_bt.read_text()) == original
    # an untouched value survives the YAML round trip with its type
    assert overlay['controller_server']['ros__parameters']['enable_stamped_cmd_vel'] is False
    assert math.isclose(overlay['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'], 0.25)


def test_005_R34_invalid_factor_fails_the_launch(nav_launch, tmp_path):
    with pytest.raises(ValueError):
        nav_launch.slowed_files('p', 'b', '0', directory=str(tmp_path))


def test_005_R34_launch_declares_sim_slowdown_with_default_one(nav_launch):
    from launch.actions import DeclareLaunchArgument
    declared = {e.name: e for e in nav_launch.generate_launch_description().entities
                if isinstance(e, DeclareLaunchArgument)}
    assert 'sim_slowdown' in declared
    assert declared['sim_slowdown'].default_value[0].text == '1.0'


def test_005_R34_scaled_keys_lists_what_a_params_file_actually_has():
    from vacuum_bringup.slowdown import scaled_keys
    params = yaml.safe_load((PKG / 'config' / 'nav2_params.yaml').read_text())   # fresh: the fixture is mutated above
    assert len(scaled_keys(params)) == len(NAV_TIMEOUT_PARAMS)
    assert scaled_keys({'controller_server': {'ros__parameters': {'other': 1}}}) == []
    assert scaled_keys({}) == []


def test_005_R34_count_bt_timeouts():
    from vacuum_bringup.slowdown import count_bt_timeouts
    assert count_bt_timeouts('<Timeout msec="5"><Timeout msec="7"/><Wait msec="1"/>') == 2
    assert count_bt_timeouts('<Wait msec="1"/>') == 0


def test_005_R34_custom_params_file_with_nothing_to_scale_is_logged(nav_launch, tmp_path):
    custom = tmp_path / 'custom.yaml'
    custom.write_text('controller_server: {ros__parameters: {other: 1}}\n')
    bt = tmp_path / 'bt.xml'
    bt.write_text('<root><Wait msec="1"/></root>')
    lines = []
    nav_launch.slowed_files(str(custom), str(bt), '3', directory=str(tmp_path / 'out'), log=lines.append)
    assert any('params_file' in line and 'nothing' in line for line in lines), lines
    assert any('bt_xml' in line and 'Timeout' in line for line in lines), lines


def test_005_R34_shipped_files_log_no_warning(nav_launch, tmp_path):
    lines = []
    nav_launch.slowed_files(str(PKG / 'config' / 'nav2_params.yaml'), str(PKG / 'config' / 'nav_bt.xml'), '3',
                            directory=str(tmp_path), log=lines.append)
    assert lines == []


def test_005_R34_overlay_directory_is_removed_at_exit(nav_launch, monkeypatch):
    import atexit
    import os
    registered = []
    monkeypatch.setattr(atexit, 'register', lambda fn, *a, **k: registered.append((fn, a, k)))
    out_params, _ = nav_launch.slowed_files(str(PKG / 'config' / 'nav2_params.yaml'),
                                            str(PKG / 'config' / 'nav_bt.xml'), '2')
    directory = os.path.dirname(out_params)
    assert os.path.isdir(directory) and registered
    fn, a, k = registered[0]
    fn(*a, **k)
    assert not os.path.exists(directory)


def test_005_R34_a_caller_given_directory_is_not_removed(nav_launch, monkeypatch, tmp_path):
    import atexit
    registered = []
    monkeypatch.setattr(atexit, 'register', lambda fn, *a, **k: registered.append(fn))
    nav_launch.slowed_files(str(PKG / 'config' / 'nav2_params.yaml'), str(PKG / 'config' / 'nav_bt.xml'), '2',
                            directory=str(tmp_path))
    assert registered == []
