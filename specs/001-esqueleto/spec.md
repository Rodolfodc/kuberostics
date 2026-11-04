# 001: Esqueleto andante

Status: rascunho para revisão · Prazo: 10/11/2026 · Marco: sistema completo rodando com stubs

## Objetivo

Colocar no ar, com um único comando, o orquestrador do Luciano, o `twist_mux`, o supervisor de submodos e um
stub para cada módulo, todos integrados por meio do contrato da spec 000. O robô deve se mover no CoppeliaSim
e o ciclo travamento → fuga → volta à cobertura deve funcionar com dados falsos. Toda spec seguinte troca um
stub por uma implementação real sem mexer no restante.

## Fora do escopo

- Navegação real, SLAM, YOLO, agenda, interface gráfica, RL e nuvem.
- Substituição do `myrobot_server` do Luciano (D-08).

## Cenários de uso

1. **Ciclo feliz.** Com o sistema no ar, o usuário chama `start`. O orquestrador vai para CLEANING, o supervisor
   publica COVERING e o robô anda pelo stub de cobertura.
2. **Travamento.** Durante a cobertura, o stub do detector publica um `StuckEvent`. O supervisor muda para
   ESCAPING e chama a ação de fuga. A fuga termina com sucesso e o supervisor volta a COVERING.
3. **Fuga falha.** A fuga estoura o tempo limite. O supervisor publica ESCAPE_FAILED, ativa o lock de parada e
   emite uma notificação de erro. O robô fica parado.
4. **Teleoperação.** Em qualquer modo, um comando de teleoperação tem precedência sobre os atuadores automáticos.
   O lock de parada continua tendo precedência sobre a teleoperação.

## Requisitos (EARS)

| ID | Requisito | Verificação |
|---|---|---|
| 001-R01 | O sistema deve subir com um único `ros2 launch aspirador_bringup skeleton.launch.py`, contendo os nós do Luciano, o `twist_mux`, o supervisor e os stubs. | launch_testing |
| 001-R02 | Quando a subida terminar, o lifecycle manager do Luciano e o `lifecycle_manager_core` devem responder `true` em `is_active`. | launch_testing |
| 001-R03 | O sistema deve ter exatamente um publicador em `/myRobot/cmd_vel`, e ele deve ser o `twist_mux`. | contrato |
| 001-R04 | Cada interface marcada com `since: "001"` ou `frozen: true` em `topics.yaml` deve existir com o tipo e o QoS declarados. | contrato |
| 001-R05 | Enquanto o modo do orquestrador não for CLEANING nem AUTODOCKING, o supervisor deve publicar o submodo NONE. | unitário |
| 001-R06 | Quando o modo passar a CLEANING, o supervisor deve publicar COVERING em até 0,5 s e enviar um objetivo para `/myRobot/cover_room`. | launch_testing |
| 001-R07 | Quando receber um `StuckEvent` com submodo COVERING ou com modo AUTODOCKING, o supervisor deve cancelar a ação em curso, publicar ESCAPING em até 0,5 s e enviar um objetivo para `/myRobot/escape`. | launch_testing |
| 001-R08 | Quando a ação de fuga terminar com sucesso, o supervisor deve retornar ao submodo anterior ao travamento. | launch_testing |
| 001-R09 | Se a ação de fuga falhar ou exceder `escape_timeout_s` (padrão 20 s), o supervisor deve publicar ESCAPE_FAILED, publicar `true` em `/myRobot/mux/lock_stop` e emitir uma `Notification` de severidade ERROR com código `ESCAPE_FAILED`. | launch_testing |
| 001-R10 | Enquanto `/myRobot/mux/lock_stop` for `true`, o `twist_mux` deve publicar velocidade zero, qualquer que seja a entrada. | launch_testing |
| 001-R11 | Enquanto houver mensagens em `/myRobot/mux/teleop` dentro do tempo limite da entrada, o `twist_mux` deve repassar a teleoperação e ignorar as entradas de prioridade menor. | launch_testing |
| 001-R12 | O sistema deve remapear a saída do `cleaning_server` do Luciano para `/myRobot/legacy/cleaning_cmd_vel` e a do `autodocking_server` para `/myRobot/mux/docking`, sem alterar o código desses nós. | contrato |
| 001-R13 | Onde o CoppeliaSim estiver em execução, a ponte de rodas deve converter `/myRobot/cmd_vel` em velocidades das rodas do robô na cena. | manual (vídeo curto) |
| 001-R14 | O repositório deve conter um script que mede passos de simulação por segundo da cena *House* em modo síncrono e sem interface gráfica, e o resultado deve ser registrado na decisão D-11. | manual |
| 001-R15 | Quando houver push, a integração contínua deve executar `colcon build` e `colcon test` em contêiner `ros:jazzy` sem CoppeliaSim. | CI |
| 001-R16 | O stub `escape_server` deve aceitar o parâmetro `outcome` (`succeed`, `fail`, `hang`) para que os cenários 2 e 3 sejam testáveis sem simulador. | unitário |

## Critério de pronto

- Todos os testes automáticos de 001-R01 a 001-R12 e 001-R16 passando na CI.
- Vídeo de até 1 minuto mostrando os cenários 1 a 4 no CoppeliaSim.
- D-11 registrada em `specs/000-sistema/spec.md` com a medição de passos/s.
