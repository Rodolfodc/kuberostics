"""T006: testes de SubmodeMachine, uma linha da tabela de transições por caso.

Cada teste cita o requisito que verifica (Art. VIII). Sem ROS: a lógica é Python puro.
"""
import pytest

from aspirador_supervisor.submode_machine import (
    COVERING, ESCAPE_FAILED, ESCAPING, NONE, Effect, SubmodeMachine,
)


def kinds(effects):
    return [e.kind for e in effects]


def machine_covering():
    m = SubmodeMachine()
    m.on_mode('CLEANING')
    return m


# --- 001-R05 ---------------------------------------------------------------
@pytest.mark.parametrize('mode', ['IDLE', 'CHARGING', 'CHARGED', 'DESCONHECIDO', ''])
def test_001_R05_modo_fora_de_cleaning_e_autodocking_fica_none(mode):
    m = SubmodeMachine()
    m.on_mode(mode)
    assert m.submode == NONE


# --- 001-R06 ---------------------------------------------------------------
def test_001_R06_cleaning_inicia_covering_e_envia_cover_room():
    m = SubmodeMachine()
    fx = m.on_mode('CLEANING')
    assert m.submode == COVERING
    assert fx == [Effect('send_cover_room')]


def test_001_R06_modo_republicado_e_idempotente():
    m = machine_covering()
    assert m.on_mode('CLEANING') == []
    assert m.submode == COVERING


def test_001_R05_sair_de_cleaning_cancela_e_volta_a_none():
    m = machine_covering()
    fx = m.on_mode('IDLE')
    assert m.submode == NONE
    assert kinds(fx) == ['cancel_actions']


# --- 001-R07 ---------------------------------------------------------------
def test_001_R07_stuck_em_covering_cancela_e_escapa():
    m = machine_covering()
    fx = m.on_stuck()
    assert m.submode == ESCAPING
    assert kinds(fx) == ['cancel_actions', 'send_escape']


def test_001_R07_stuck_em_autodocking_escapa_com_retorno_none():
    m = SubmodeMachine()
    m.on_mode('AUTODOCKING')
    assert m.submode == NONE
    fx = m.on_stuck()
    assert m.submode == ESCAPING
    assert 'send_escape' in kinds(fx)
    m.on_escape_result(success=True)
    assert m.submode == NONE


@pytest.mark.parametrize('mode', ['IDLE', 'CHARGING', 'CHARGED'])
def test_001_R07_stuck_fora_de_covering_e_autodocking_e_ignorado(mode):
    m = SubmodeMachine()
    m.on_mode(mode)
    assert m.on_stuck() == []
    assert m.submode == NONE


def test_001_R07_stuck_durante_escaping_e_ignorado():
    m = machine_covering()
    m.on_stuck()
    assert m.on_stuck() == []
    assert m.submode == ESCAPING


# --- 001-R08 ---------------------------------------------------------------
def test_001_R08_fuga_com_sucesso_retorna_a_covering_e_reenvia_cover_room():
    m = machine_covering()
    m.on_stuck()
    fx = m.on_escape_result(success=True)
    assert m.submode == COVERING
    assert fx == [Effect('send_cover_room')]


# --- 001-R09 ---------------------------------------------------------------
def test_001_R09_fuga_falha_trava_e_notifica():
    m = machine_covering()
    m.on_stuck()
    fx = m.on_escape_result(success=False)
    assert m.submode == ESCAPE_FAILED
    assert Effect('lock_stop', True) in fx
    notes = [e for e in fx if e.kind == 'notify']
    assert len(notes) == 1
    assert notes[0].value == ('ERROR', 'ESCAPE_FAILED')


def test_001_R09_escape_timeout_do_supervisor_equivale_a_falha():
    m = machine_covering()
    m.on_stuck()
    fx = m.on_escape_timeout()
    assert m.submode == ESCAPE_FAILED
    assert Effect('lock_stop', True) in fx


def test_001_R09_reset_libera_lock_e_volta_a_none():
    m = machine_covering()
    m.on_stuck()
    m.on_escape_result(success=False)
    fx = m.reset()
    assert m.submode == NONE
    assert fx == [Effect('lock_stop', False)]


def test_001_R09_reset_sem_falha_nao_faz_nada():
    m = machine_covering()
    assert m.reset() == []
    assert m.submode == COVERING


def test_001_R09_cleaning_nao_reinicia_enquanto_escape_failed():
    m = machine_covering()
    m.on_stuck()
    m.on_escape_result(success=False)
    assert m.on_mode('CLEANING') == []
    assert m.submode == ESCAPE_FAILED


# --- robustez (resultado atrasado depois de cancelar) ----------------------
def test_resultado_de_fuga_fora_de_escaping_e_ignorado():
    m = machine_covering()
    assert m.on_escape_result(success=False) == []
    assert m.submode == COVERING


def test_001_R05_sair_de_cleaning_durante_escaping_cancela_a_fuga():
    m = machine_covering()
    m.on_stuck()
    fx = m.on_mode('IDLE')
    assert m.submode == NONE
    assert kinds(fx) == ['cancel_actions']
    assert m.on_escape_result(success=True) == []   # resultado atrasado não reanima nada
