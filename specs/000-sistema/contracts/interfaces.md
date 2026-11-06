# Tipos novos do pacote `aspirador_interfaces` (spec 001)

Rascunho dos arquivos `.msg` e `.action`. A tarefa T002 da spec 001 copia este conteúdo para o pacote.

## msg/Submode.msg

```
# Submodo publicado pelo supervisor dentro do modo do orquestrador (D-01).
string NONE=NONE
string EXPLORING=EXPLORING
string COVERING=COVERING
string GOTO_ROOM=GOTO_ROOM
string ESCAPING=ESCAPING
string ESCAPE_FAILED=ESCAPE_FAILED

builtin_interfaces/Time stamp
string mode        # cópia do último /myRobot/behaviour/mode recebido
string submode     # uma das constantes acima
string reason      # motivo da última transição, para log e interface
```

## msg/StuckEvent.msg

```
string NO_PROGRESS=NO_PROGRESS   # comando de movimento sem deslocamento correspondente
string BUMPER=BUMPER

builtin_interfaces/Time stamp
string cause
geometry_msgs/Pose2D pose        # pose no mapa no instante da detecção
float32[] scan_ranges            # cópia do laser no instante (insumo para o treino da spec 008)
```

## msg/Notification.msg

```
uint8 INFO=0
uint8 WARN=1
uint8 ERROR=2

builtin_interfaces/Time stamp
uint8 severity
string source      # nome do nó
string code        # identificador estável, ex.: ESCAPE_FAILED
string text        # mensagem para o usuário
```

## msg/OffloadStatus.msg (spec 009a)

```
uint8 CLOSED=0      # usando o detector da nuvem
uint8 OPEN=1        # usando o detector local
uint8 HALF_OPEN=2   # usando o local enquanto a nuvem prova estabilidade

builtin_interfaces/Time stamp
uint8 state
float32 last_cloud_age_s       # idade da última resposta da nuvem, no relógio do robô
uint32 dropped_stale           # respostas descartadas por idade
uint32 dropped_watermark       # descartadas por carimbo já coberto (duplicata ou fora de ordem)
uint32 dropped_tf              # descartadas pelo projector por falta de TF no carimbo
```

## action/Escape.action

```
# Goal
StuckEvent event
float32 timeout_s
---
# Result
bool success
float32 duration_s
string strategy     # "scripted" ou "rl", para comparar implementações
---
# Feedback
float32 elapsed_s
```

## action/CoverRoom.action

```
# Goal
string room_id      # vazio = cômodo atual
uint8 passes
---
# Result
bool success
float32 covered_ratio
---
# Feedback
float32 covered_ratio
```
