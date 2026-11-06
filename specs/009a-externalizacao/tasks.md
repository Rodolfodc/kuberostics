# 009a: Tarefas

## Fase 0: preparação

- [ ] **T901** Adicionar `OffloadStatus` a `aspirador_interfaces`. (009a-R14)
- [ ] **T902** [P] Medir o transporte robô ↔ K3s com `rmw_zenoh` e com `zenoh-bridge-ros2dds`: latência e taxa com imagem RGB comprimida a 2 Hz. Decidir Q-06. (009a-R03)
- [ ] **T903** [P] Ler o nó semântico do 2025S2 e registrar o formato exato de `/detections_markers` que ele espera. (009a-R13)

## Fase 1: testes primeiro

- [ ] **T904** Testes pytest de `selector_core`: filtros de idade e de marca d'água, cada transição da tabela e a histerese, com relógio simulado. (009a-R05 a R10)
- [ ] **T905** [P] Teste do `projector` com TF sintético: transformação no carimbo da foto, descarte sem TF. (009a-R12)

## Fase 2: implementação

- [ ] **T906** `detector`: YOLO configurável por parâmetro, `header` copiado, array vazio quando não detecta nada. (009a-R02, R04)
- [ ] **T907** `selector_core` e o nó `detection_selector`, até a T904 passar. Publicar o status e as notificações. (009a-R05 a R11, R14)
- [ ] **T908** `projector` com cache de profundidade por carimbo, até a T905 passar. (009a-R12, R13)
- [ ] **T909** `image_throttle`: compressão e taxa configurável. (009a-R03, R11)
- [ ] **T910** Dockerfile do detector com `ARG ROS_DISTRO`, build na matriz da CI. (009a-R17, R18)
- [ ] **T911** Terraform (`infra/modules/workloads`) com Deployment, `livenessProbe` e o transporte escolhido em Q-06. (009a-R17)
- [ ] **T912** Launch com `offload:=true|false`. (009a-R01)

## Fase 3: verificação

- [ ] **T913** `tools/chaos/kill_detector.sh` e a análise do rosbag. Registrar os tempos e contadores para o relatório. (009a-R15, R16)
- [ ] **T914** Rodar T913 em Jazzy e em Lyrical. (009a-R18)
