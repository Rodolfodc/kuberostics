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
| 009a-externalizacao | Externalização do detector YOLO para o K3s, com circuit breaker e fallback local (arquitetura de referência: FogROS2) | 3 + inovação (sugestão do professor) |
| 009b-nuvem | Ponte MQTT, K3s, ciclo de retreino | inovação |
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
| D-11 | (reservada para a medição de passos/s, tarefa T004 da spec 001) | |
| D-12 | O sistema roda em Jazzy e em Lyrical. Cada distribuição tem seu contêiner (`docker/jazzy`, `docker/lyrical`) e a CI roda as duas. | Constituição, Art. X. Todas as dependências de terceiros já têm release nas duas: `navigation2` (1.3.14 / 1.5.3), `slam_toolbox` (2.8.5 / 2.10.0), `twist_mux` (4.5.0 / 4.5.1), `teleop_twist_keyboard`, `rmw_cyclonedds`, `rmw_zenoh` e `vision_msgs`. Fonte: `ros/rosdistro`, 05/11/2026. |
| D-13 | O contrato de movimento é `geometry_msgs/Twist` sem carimbo de tempo, como a base do Luciano espera. Ficam explícitos `use_stamped: false` no `twist_mux` e `enable_stamped_cmd_vel: false` em todo nó do Nav2 que publique velocidade. | O `twist_mux` 4.5 usa `use_stamped = true` por padrão nas duas distribuições. O Nav2 usa `enable_stamped_cmd_vel = false` no Jazzy e `true` no Lyrical. Confiar no padrão quebraria a base num caso e o Nav2 no outro. Fecha a Q-02. |
| D-14 | O CoppeliaSim conversa com o ROS apenas pela ZeroMQ Remote API (nós Python da ponte). O plugin simROS2 não é usado. | A ponte por ZeroMQ não depende da distribuição ROS. O simROS2 precisaria ser compilado para cada distribuição e cada versão do CoppeliaSim. O simulador roda no hospedeiro e os contêineres ROS o acessam pela rede. |
| D-15 | A externalização de nós para a nuvem é feita com Kubernetes nativo (K3s): uma imagem por distribuição, um Deployment e um seletor no robô. O FogROS2 (Ichnowski et al., ICRA 2023) e o KubeROS (Zhang, Wurll e Hein, ICRA 2023) entram como arquitetura de referência; nenhum dos dois é usado como dependência. | O FogROS2 só tem versão para Humble, cria Pods avulsos privilegiados sem reinício e instala o ROS por SSH a cada execução. Portá-lo para Jazzy e Lyrical custaria de 4 a 7 dias. O porte completo fica como trabalho futuro, de preferência como contribuição ao repositório original. |
| D-16 | A detecção é dividida em dois nós: o `detector` (YOLO: imagem RGB → `vision_msgs/Detection2DArray` no referencial da câmera, copiando o `header` da imagem) e o `projector` (sempre no robô: detecção 2D + profundidade + TF no carimbo da foto → posição 3D no mapa). | Só a imagem RGB comprimida sobe para a nuvem; a profundidade e o TF ficam no robô. O `header` copiado é o eco do carimbo. O `projector` consulta o TF no instante da foto e mantém a saída `/detections_markers` que o nó semântico do 2025S2 consome, que segue inalterado. |
| D-17 | `StuckEvent.pose` usa `geometry_msgs/Pose`, não `Pose2D`. | `Pose2D` existe no Jazzy e foi removido do Lyrical (verificado em `geometry_msgs/msg` nas duas imagens, 06/11/2026). Art. X exige o mesmo contrato nas duas. O robô é planar: z = 0 e o yaw vem do quaternion. |
| D-18 | O código é modular e segue boas práticas: lógica pura em módulos Python sem ROS, nós ROS finos que só ligam a lógica a tópicos, e dependências injetadas pelo invocador. | Facilita o teste sem simulador (Art. V) e a troca de módulos (Art. III). |
| D-19 | Componentes reutilizáveis (controladores, filtros, planejadores, detectores) são caixas de ferramentas: a classe só faz o seu cálculo, sem publicar, sem ler tópicos e sem guardar política. Quem a invoca decide configuração, limites, quando chamar e o que fazer com o resultado. | O invocador conhece o contexto (mapa, modo, segurança) e a classe fica testável e reutilizável. Exemplo de estilo: um `PID` que só calcula `u` a partir de erro e `dt`. |
| D-20 | Todo nó e módulo registra logs e emite telemetria sempre que possível: `rclpy` logger com níveis, transições de estado e eventos (stuck, escape, lock) sempre logados, e métricas periódicas em `/myRobot/telemetry/<nó>` ou `diagnostic_msgs/DiagnosticArray`. A lógica pura recebe um logger e/ou um callback de métricas, não importa ROS. | Sem isso não há como depurar no simulador, medir D-11 nem alimentar o treino (spec 008). |

## 6. Questões em aberto

- Q-01: versão do CoppeliaSim e caminho da cena *House*. Os nomes de objetos (`/myRobot/leftMotor`)
  precisam bater com a cena do 2025S2.
- ~~Q-02~~: fechada por D-13.
- Q-04: o `my_robot_pkg` do Luciano foi escrito para Jazzy. É preciso confirmar que ele compila e roda em Lyrical
  (rclpy 10, Python 3.14). Se não rodar, a correção entra como sobreposição no nosso repositório, nunca como edição do código dele.
- Q-05: dependências pip na versão 3.14 do Python, usadas no Lyrical: `torch`, `ultralytics` e `stable-baselines3`.
  Se alguma não tiver wheel, o treino de RL roda num ambiente Python separado do ROS, e só a inferência entra no nó.
- Q-06: transporte robô ↔ nuvem. Duas opções:
  - (a) `rmw_zenoh` no grafo inteiro, com o roteador do robô ligado ao roteador do cluster (0.2.13 no Jazzy, 0.10.8 no Lyrical);
  - (b) `zenoh-bridge-ros2dds` (1.9.0), mantendo o grafo local em DDS com CycloneDDS e uma lista explícita (*allow-list*) de tópicos que sobem.

  Nenhuma das duas implementa os eventos de QoS de *Deadline* e *Liveliness* do jeito que precisamos. Por isso a spec 009a implementa idade, heartbeat e limiares na aplicação, independentemente do transporte.
  Decidir após medir as duas na semana 3.
- Q-07 (futuro): protocolo de comunicação robô ↔ nuvem, ou seja, formato, versionamento, autenticação e a política de quando sobe o quê. Fica fora do prazo da disciplina. A spec 009a define só o mínimo necessário.
- Q-03: odometria. O 2025S2 usa `/myrobot/odom` (verdade do simulador) e rf2o (odometria por laser).
  Definir na spec 003 qual alimenta o SLAM.
