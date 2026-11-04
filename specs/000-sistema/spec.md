# 000: Especificação do sistema

Status: rascunho para revisão · 2026-11-04

Este documento fixa as fronteiras do sistema, o contrato herdado e as decisões que valem para todas as
especificações de funcionalidade (001 em diante). Ele não descreve comportamento de módulo em detalhe;
isso fica nas especificações 002 a 008.

## 1. Contexto

Robô aspirador diferencial (cerca de 32 cm) na cena *House* do CoppeliaSim. Sensores: laser 2D,
câmera RGB-D e bumper. O sistema cumpre os seis módulos do enunciado da disciplina. A inovação está em:

- uma política de fuga aprendida por reforço;
- um serviço em nuvem emulado em K3s.

## 2. Atores

| Ator | Interação |
|---|---|
| Usuário | Teleopera, clica objetivos, edita cômodos e agenda, recebe notificações. |
| Orquestrador do Luciano | Decide o modo operacional (inalterado). |
| CoppeliaSim | Fornece sensores e executa as velocidades das rodas. |
| Serviço em nuvem (opcional) | Guarda histórico, retreina a política e publica pesos aprovados. |

## 3. Mapa de especificações

| Spec | Conteúdo | Módulo do enunciado |
|---|---|---|
| 001-esqueleto | Integração de ponta a ponta com stubs | todos |
| 002-controle | Teleoperação, objetivo clicado | 1 |
| 003-mapeamento | Exploração, SLAM, segmentação, rótulos | 2 |
| 004-reconhecimento | YOLO, posição 3D, ontologia | 3 |
| 005-cobertura-e-fuga | Caminho global, cobertura, desvio, travamento, fuga por roteiro fixo | 5 |
| 006-agenda | Agenda manual e autônoma | 4 |
| 007-interface | Mapa, edição de cômodos, notificações | 6 |
| 008-fuga-rl | Ambiente Gymnasium, treino, validação, troca de implementação | 5 (inovação) |
| 009-nuvem | Ponte MQTT, K3s, ciclo de retreino | inovação |
| 010-agente-llm | Reclamação em especificação de armadilha | opcional |

## 4. Contrato herdado (congelado)

Fonte: `lucianovk/IA368II`, commit `51a5fa2c816c7ee712c963843e5d747b3ed2a98f`.
A lista completa, legível por máquina, está em `contracts/topics.yaml`.

**Orquestrador (`myrobot_bringup`)**

- Nós gerenciados: `myrobot`, `behaviour_server`, `cleaning_server`, `autodocking_server`,
  `charging_server`, `charged_server`.
- Modo: `/myRobot/behaviour/mode` (`std_msgs/String`), republicado a cada 0,2 s.
  Valores: CLEANING, AUTODOCKING, CHARGING, CHARGED, IDLE.
  Os nós do Luciano tratam qualquer valor desconhecido como IDLE.
- Comando do usuário: `/myRobot/start_stop` (`myrobot_interfaces/srv/StartStop`, `"start"` ou `"stop"`).
- Telemetria: `/myRobot/battery_state`, `/myRobot/charging_base/strengthSignal`,
  `/myRobot/charging_base/relativeAngle`.
- QoS de todos os tópicos: profundidade 1, confiável, volátil.

**Projeto 2025S2 (`my_robot_pkg`, reuso nos módulos 2 e 3)**

- Ponte para o CoppeliaSim pela ZeroMQ Remote API, em nós comuns (não lifecycle).
- Publica `/scan`, `/camera`, `/camera_depth` e `/myrobot/odom` (minúsculo).
- Assina `/cmd_vel`.
- Os nós de percepção publicam `/topologic_map`, `/detections_markers` e `/semantic_map`.

## 5. Decisões de arquitetura

| ID | Decisão | Justificativa |
|---|---|---|
| D-01 | Os modos de cima continuam sendo do orquestrador; os comportamentos novos são **submodos**, publicados pelo `supervisor` em `/myRobot/behaviour/submode`. | Constituição, Art. II. Os nós do Luciano tratam modo desconhecido como IDLE. Máquina de estados hierárquica (Harel, 1987). |
| D-02 | Comportamentos com resultado (fuga, cobertura de cômodo, ida a um ponto) são **ações ROS 2** chamadas pelo supervisor. | O tópico de modo não tem como informar sucesso ou falha; a ação tem resultado, retorno parcial e cancelamento. |
| D-03 | Somente o `twist_mux` publica em `/myRobot/cmd_vel`, com prioridades e um lock de parada. | Constituição, Art. VI. Evita que dois atuadores comandem a base ao mesmo tempo. |
| D-04 | A saída do stub `cleaning_server` do Luciano é remapeada para `/myRobot/legacy/cleaning_cmd_vel`, que nenhum nó assina. O nó continua no ar porque o lifecycle manager dele o espera. | Se entrasse no mux, mesmo com prioridade mínima, o stub giraria o robô sempre que os atuadores reais ficassem em silêncio. |
| D-05 | O `autodocking_server` do Luciano é remapeado para a entrada `docking` do mux. | É o atuador de doca até que a spec de doca o substitua por nome. |
| D-06 | A ponte de rodas do 2025S2 (`my_robot_cmd_vel_node`) é reaproveitada com `/cmd_vel` remapeado para `/myRobot/cmd_vel`. | Reuso sem alteração de código. |
| D-07 | Os tópicos de sensor mantêm os nomes do 2025S2 (`/scan`, `/camera`, `/camera_depth`). Novos módulos os consomem por parâmetro de nome. | Os módulos 2 e 3 reaproveitam esses nós sem mudança. |
| D-08 | Até a spec de doca, o `myrobot_server` do Luciano continua simulando bateria e sinal da base. | Mantém o ciclo do orquestrador funcionando desde a semana 1. A substituição por um nó ligado ao CoppeliaSim fica para depois. |
| D-09 | Dois lifecycle managers próprios: `lifecycle_manager_core` (supervisor, stubs, módulos) e `lifecycle_manager_optional` (ponte MQTT). O lifecycle manager do Luciano permanece como ele o configurou. | Constituição, Art. VII. A ausência de um opcional não trava o núcleo. |
| D-10 | O Luciano é trazido por `vcs import` de `deps.repos`, fixado por commit. | Reprodutibilidade; o código dele não é copiado nem editado. |

## 6. Questões em aberto

- Q-01: versão do CoppeliaSim e caminho da cena *House*. Os nomes de objetos (`/myRobot/leftMotor`)
  precisam bater com a cena do 2025S2.
- Q-02: `twist_mux` no Jazzy publica `Twist` ou `TwistStamped`? A base espera `Twist`. Verificar na tarefa T007 da spec 001.
- Q-03: odometria. O 2025S2 usa `/myrobot/odom` (verdade do simulador) e rf2o (odometria por laser).
  Definir na spec 003 qual alimenta o SLAM.
