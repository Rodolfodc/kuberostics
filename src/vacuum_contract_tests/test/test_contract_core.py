"""Unit tests of the pure contract logic and of topics.yaml itself. No ROS, no graph (D-18)."""
import copy

import pytest

from vacuum_contract_tests.contract import (
    applies, build_expectations, check_named_type, check_topic, contract_problems, find_contract,
    load_contract, normalize_type, Endpoint, QosSpec, SIM_OWNERS)

IA368II = QosSpec('reliable', 'volatile', 1)


@pytest.fixture(scope='module')
def contract():
    return load_contract(find_contract())


def exp_by_name(contract, name):
    return next(e for e in build_expectations(contract) if e.name == name)


# ---- topics.yaml itself ------------------------------------------------------------------------------------

def test_001_R04_contract_is_well_formed(contract):
    """001-R04: the contract the tests are generated from has no unknown QoS profile, bad name or type."""
    assert contract_problems(contract) == []


def test_001_R04_scope_is_since_001_or_no_since(contract):
    """001-R04: exactly the interfaces with `since: "001"` or no `since` are expected, frozen or not."""
    names = {e.name for e in build_expectations(contract)}
    assert '/myRobot/cmd_vel' in names                       # frozen
    assert '/myRobot/behaviour/submode' in names             # since 001
    assert '/myRobot/mux/nav' not in names                   # since 005
    assert '/myRobot/perception/detections/local' not in names   # since 009a
    assert '/detections_markers' not in names                # frozen but since 009a: not in the skeleton
    for e in build_expectations(contract):
        assert e.since in ('', '001')   # `since` missing is carried as ''


def test_001_R03_cmd_vel_is_twist_with_one_publisher(contract):
    """001-R03: /myRobot/cmd_vel is an unstamped Twist (D-13), one publisher, the owner is twist_mux."""
    e = exp_by_name(contract, '/myRobot/cmd_vel')
    assert e.type == 'geometry_msgs/msg/Twist'
    assert e.max_publishers == 1
    assert e.owner == 'twist_mux'


def test_001_R12_legacy_cleaning_output_is_a_sink(contract):
    """001-R12: the cleaning_server output goes to a sink Twist topic owned by cleaning_server."""
    e = exp_by_name(contract, '/myRobot/legacy/cleaning_cmd_vel')
    assert e.sink and e.owner == 'cleaning_server'
    assert e.type == 'geometry_msgs/msg/Twist'


def test_001_R12_docking_goes_to_the_mux_input(contract):
    """001-R12: the autodocking_server output is the mux docking input."""
    e = exp_by_name(contract, '/myRobot/mux/docking')
    assert e.owner == 'autodocking_server' and not e.sink
    assert e.type == 'geometry_msgs/msg/Twist'


def test_001_R04_services_and_actions_are_expected(contract):
    kinds = {(e.kind, e.name) for e in build_expectations(contract)}
    assert ('service', '/myRobot/supervisor/reset') in kinds
    assert ('action', '/myRobot/cover_room') in kinds
    assert ('action', '/myRobot/escape') in kinds


def test_001_R04_exclude_owners_drops_sim_topics(contract):
    names = {e.name for e in build_expectations(contract, exclude_owners=SIM_OWNERS)}
    assert '/scan' not in names and '/camera' not in names
    assert '/myRobot/cmd_vel' in names


def test_ia368ii_profile_is_reliable_volatile_depth_1(contract):
    """001-R04 (QoS): the inherited QoS (QoSProfile(depth=1)) resolves to what the header of topics.yaml says."""
    assert exp_by_name(contract, '/myRobot/cmd_vel').qos == IA368II


# ---- contract_problems detects errors -------------------------------------------------------------------------

def test_001_R04_contract_problems_detects_bad_entries(contract):
    bad = copy.deepcopy(contract)
    bad['topics'].append({'name': 'relative', 'type': 'std_msgs/String', 'qos': 'nope'})
    bad['topics'].append(dict(bad['topics'][0]))
    problems = '\n'.join(contract_problems(bad))
    assert 'not absolute' in problems
    assert 'malformed type' in problems
    assert 'unknown qos profile' in problems
    assert 'duplicate' in problems


# ---- check_topic ----------------------------------------------------------------------------------------------

def twist_exp(contract):
    return exp_by_name(contract, '/myRobot/cmd_vel')


def mux_pub(qos=IA368II, type_='geometry_msgs/msg/Twist', node='twist_mux'):
    return Endpoint(node, type_, qos)


def test_001_R04_missing_topic_fails(contract):
    assert check_topic(twist_exp(contract), [], []) == ['/myRobot/cmd_vel: not in the graph']


def test_001_R03_single_twist_mux_publisher_conforms(contract):
    assert check_topic(twist_exp(contract), [mux_pub()], [Endpoint('wheel', 'geometry_msgs/msg/Twist')]) == []


def test_001_R03_two_publishers_fail(contract):
    problems = check_topic(twist_exp(contract), [mux_pub(), mux_pub(node='autodocking_server')], [])
    assert any('2 publishers' in p for p in problems)
    assert any('only twist_mux may publish' in p for p in problems)


def test_001_R03_wrong_publisher_node_fails(contract):
    problems = check_topic(twist_exp(contract), [mux_pub(node='cleaning_server')], [])
    assert any('only twist_mux may publish' in p for p in problems)


def test_001_R03_stamped_twist_fails(contract):
    """D-13: a TwistStamped on /myRobot/cmd_vel (Lyrical default of twist_mux) violates the contract."""
    problems = check_topic(twist_exp(contract), [mux_pub(type_='geometry_msgs/msg/TwistStamped')], [])
    assert any('TwistStamped' in p for p in problems)


def test_001_R04_wrong_qos_fails(contract):
    problems = check_topic(twist_exp(contract), [mux_pub(qos=QosSpec('best_effort', 'volatile', 1))], [])
    assert any('QoS' in p for p in problems)
    problems = check_topic(twist_exp(contract), [mux_pub(qos=QosSpec('reliable', 'volatile', 10))], [])
    assert any('QoS' in p for p in problems)


def test_001_R04_subscriber_only_topic_exists(contract):
    """A mux input with only twist_mux listening exists in the graph and has the right type."""
    e = exp_by_name(contract, '/myRobot/mux/teleop')
    assert check_topic(e, [], [Endpoint('twist_mux', 'geometry_msgs/msg/Twist')]) == []


def test_001_R12_sink_with_only_owner_publishing_conforms(contract):
    e = exp_by_name(contract, '/myRobot/legacy/cleaning_cmd_vel')
    assert check_topic(e, [Endpoint('cleaning_server', 'geometry_msgs/msg/Twist', IA368II)], []) == []


def test_001_R12_sink_with_subscriber_fails(contract):
    e = exp_by_name(contract, '/myRobot/legacy/cleaning_cmd_vel')
    problems = check_topic(e, [Endpoint('cleaning_server', 'geometry_msgs/msg/Twist', IA368II)],
                           [Endpoint('twist_mux', 'geometry_msgs/msg/Twist')])
    assert any('has subscribers' in p for p in problems)


def test_001_R12_sink_without_publisher_fails(contract):
    """The remapping is not in place if the sink topic has only a subscriber or nothing publishing."""
    e = exp_by_name(contract, '/myRobot/legacy/cleaning_cmd_vel')
    problems = check_topic(e, [], [Endpoint('x', 'geometry_msgs/msg/Twist')])
    assert any('no publisher' in p for p in problems)


def test_001_R12_cleaning_server_still_on_cmd_vel_fails(contract):
    """If the remap is missing, cleaning_server publishes on /myRobot/cmd_vel: caught by the 001-R03 check."""
    problems = check_topic(twist_exp(contract), [mux_pub(), mux_pub(node='cleaning_server')], [])
    assert problems


def test_001_R04_owner_star_allows_any_publisher(contract):
    e = exp_by_name(contract, '/myRobot/notifications')
    assert check_topic(e, [Endpoint('anything', 'vacuum_interfaces/msg/Notification',
                                    QosSpec('reliable', 'volatile', 10))], []) == []


# ---- services / actions / helpers -----------------------------------------------------------------------------

def test_001_R04_named_type_checks(contract):
    """001-R04: services and actions are checked for existence and declared type."""
    e = exp_by_name(contract, '/myRobot/supervisor/reset')
    assert check_named_type(e, [(e.name, ['std_srvs/srv/Trigger'])]) == []
    assert 'not in the graph' in check_named_type(e, [('/other', ['std_srvs/srv/Trigger'])])[0]
    assert 'contract says' in check_named_type(e, [(e.name, ['std_srvs/srv/Empty'])])[0]


def test_001_R04_normalize_type_and_applies():
    """001-R04: scope rule (since 001 or no since, frozen irrelevant) and type spelling."""
    assert normalize_type('std_msgs/String') == 'std_msgs/msg/String'
    assert normalize_type('std_msgs/msg/String') == 'std_msgs/msg/String'
    assert applies({})
    assert applies({'frozen': True})
    assert applies({'since': '001'})
    assert not applies({'frozen': True, 'since': '009a'})
    assert not applies({'since': '005'})
