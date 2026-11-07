"""Máquina de submodos do supervisor (D-01). Python puro, sem ROS (spec 001, plano).

Entradas: modo do orquestrador, StuckEvent, resultado da ação de fuga.
Saídas: lista de `Effect` que o nó ROS executa (cancelar ações, enviar objetivos, lock, notificação).
"""
from dataclasses import dataclass
from typing import Any, List

NONE = 'NONE'
COVERING = 'COVERING'
ESCAPING = 'ESCAPING'
ESCAPE_FAILED = 'ESCAPE_FAILED'

ACTIVE_MODES = ('CLEANING', 'AUTODOCKING')


@dataclass(frozen=True)
class Effect:
    kind: str            # cancel_actions | send_cover_room | send_escape | lock_stop | notify
    value: Any = None    # lock_stop: bool; notify: (severidade, código)


class SubmodeMachine:
    def __init__(self):
        self.submode = NONE
        self.mode = ''
        self.resume_submode = NONE   # para onde voltar depois da fuga

    def on_mode(self, mode: str) -> List[Effect]:
        self.mode = mode
        if mode not in ACTIVE_MODES:
            if self.submode == NONE:
                return []
            # 001-R05: fora de CLEANING/AUTODOCKING o submodo é NONE (cancela o que estiver em curso).
            # O lock_stop não é solto aqui: só o reset do usuário o solta.
            self.submode = NONE
            self.resume_submode = NONE
            return [Effect('cancel_actions')]
        if mode == 'CLEANING' and self.submode == NONE:
            self.submode = COVERING                       # 001-R06
            return [Effect('send_cover_room')]
        return []

    def on_stuck(self) -> List[Effect]:
        if self.submode == COVERING:                      # 001-R07
            self.resume_submode = COVERING
            self.submode = ESCAPING
            return [Effect('cancel_actions'), Effect('send_escape')]
        if self.submode == NONE and self.mode == 'AUTODOCKING':
            self.resume_submode = NONE
            self.submode = ESCAPING
            return [Effect('send_escape')]
        return []

    def on_escape_result(self, success: bool) -> List[Effect]:
        if self.submode != ESCAPING:                      # resultado atrasado após cancelamento
            return []
        if success:                                       # 001-R08
            self.submode = self.resume_submode
            if self.submode == COVERING:
                return [Effect('send_cover_room')]
            return []
        return self._fail()

    def on_escape_timeout(self) -> List[Effect]:
        """O supervisor estourou `escape_timeout_s` (001-R09)."""
        if self.submode != ESCAPING:
            return []
        return [Effect('cancel_actions')] + self._fail()

    def reset(self) -> List[Effect]:
        """Serviço ~/reset do usuário: solta o lock depois de ESCAPE_FAILED."""
        if self.submode != ESCAPE_FAILED:
            return []
        self.submode = NONE
        self.resume_submode = NONE
        return [Effect('lock_stop', False)]

    def _fail(self) -> List[Effect]:
        self.submode = ESCAPE_FAILED
        return [Effect('lock_stop', True), Effect('notify', ('ERROR', 'ESCAPE_FAILED'))]
