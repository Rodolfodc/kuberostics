# 009a: Plano técnico

## Fluxo

```
                       ROBÔ                                        │        K3s
câmera RGB ─┬─▶ image_throttle (2 Hz, comprimida) ─────────────────┼──▶ detector_cloud (yolo11x)
            │                                                      │          │
            └─▶ detector_local (yolo11n, reserva quente)           │          │ Detection2DArray
                        │ Detection2DArray                         │          │ (header = header da foto)
                        ▼                                          │          ▼
                 detection_selector ◀──────────────────────────────┼──────────┘
                 (circuit breaker, idade, marca d'água)            │
                        │ /myRobot/perception/detections           │
                        ▼                                          │
câmera depth ──▶ projector (depth + TF no carimbo da foto)         │
                        │ /detections_markers                      │
                        ▼                                          │
              nó semântico do 2025S2 (inalterado)                  │
```

## Pacotes

- `aspirador_detection`: `detector` (o mesmo executável roda local ou na nuvem, com o modelo como parâmetro),
  `detection_selector` e `projector`.
- `aspirador_detection/selector_core.py`: a máquina do circuit breaker como classe Python pura, sem ROS, testada com pytest.
  Mesmo padrão do `SubmodeMachine` da spec 001.
- `deploy/k8s/`: Deployment, Service e roteador Zenoh (se Q-06 = a), ou ponte (se Q-06 = b).
- `infra/modules/workloads/`: Terraform com o provider `kubernetes` apontando para o K3s.
- `docker/detector/Dockerfile`: com `ARG ROS_DISTRO`, construído na matriz `[jazzy, lyrical]`.

## Circuit breaker

| Estado | Fonte repassada | Evento | Próximo |
|---|---|---|---|
| CLOSED | nuvem | nenhuma resposta da nuvem com idade ≤ 1,5 s por 1,5 s | OPEN (WARN) |
| OPEN | local | resposta da nuvem com idade ≤ 0,5 s | HALF_OPEN |
| HALF_OPEN | local | 3 s seguidos com toda resposta da nuvem ≤ 0,5 s | CLOSED (INFO) |
| HALF_OPEN | local | resposta da nuvem com idade > 0,5 s | OPEN |

Em todos os estados valem os filtros de idade (`max_age_s`) e de marca d'água antes de repassar.

## Por que a projeção fica no robô

- A profundidade em `float32` a 30 Hz seria cerca de 37 MB/s. Ela fica no robô, num cache curto de imagens indexado
  por carimbo (`message_filters.Cache`).
- O TF já está no robô. Consultar no carimbo da foto não exige mandar `/tf` para a nuvem nem sincronizar relógios.
- O nó semântico do 2025S2 continua recebendo `/detections_markers` como antes (Constituição, Art. II).

## Teste de falha injetada

Script `tools/chaos/kill_detector.sh`:

1. Inicia a limpeza.
2. Apaga o Pod com `kubectl delete pod`.
3. Espera o Deployment recriá-lo.
4. Repete 5 vezes.

Grava `OffloadStatus` em rosbag e calcula tempo até OPEN, tempo até CLOSED e contadores de descarte.
Também verifica a monotonicidade dos carimbos em `/myRobot/perception/detections`.

## Riscos

- Q-06 sem decisão até a medição. Os requisitos não dependem do transporte, porque a saúde é medida na aplicação.
- yolo11n na CPU do robô, mesmo a 2 Hz, pode competir com o SLAM. Mitigação: medir na semana 3 e reduzir a resolução se for preciso.
- Formato exato do `MarkerArray` esperado pelo nó semântico do 2025S2 a confirmar (T903).
