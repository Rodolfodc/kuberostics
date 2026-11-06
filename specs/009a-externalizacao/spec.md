# 009a: Externalização do detector para a nuvem

Status: rascunho para revisão · Semana 3 (18–24/11) · Depende de: 001, 004
Arquitetura de referência: FogROS2 (Ichnowski et al., ICRA 2023) e FogROS2-FT. Decisões D-15 e D-16 da spec 000.

## Objetivo

Rodar o detector YOLO grande (yolo11x) num Deployment do K3s enquanto o robô limpa, sem que o robô dependa da
nuvem. Quando a nuvem cair, ficar lenta ou voltar, o robô troca de fonte sozinho, sem detecções velhas, duplicadas
ou fora de ordem no mapa semântico. Tudo isso em Jazzy e em Lyrical.

## Fora do escopo

- Protocolo completo robô ↔ nuvem (Q-07): autenticação, versionamento, política de envio.
- Nuvem fora da rede local; TLS no transporte.
- Externalização de qualquer nó do laço de movimento (Constituição, Art. VI).

## Conceitos usados

| Problema | Solução | Origem |
|---|---|---|
| Silêncio ambíguo | O detector publica array vazio quando não vê nada; a saúde é medida pela idade da última resposta | Argumento fim a fim (Saltzer, Reed e Clark, 1984) |
| Lenta ou morta | Lentidão acima do limiar conta como falha | Resultado FLP (Fischer, Lynch e Paterson, 1985) |
| Quanto esperar | Histerese com limiares diferentes para abrir e fechar, mais uma janela de estabilidade | Circuit breaker (Nygard, *Release It!*, 2007) |
| Resposta atrasada | Eco do carimbo: idade = relógio do robô − `header.stamp` da foto | NTP (Mills); opção de carimbos do TCP (RFC 7323) |
| Duplicata ou fora de ordem | Marca d'água monotônica por carimbo de foto | *The Dataflow Model* (Akidau et al., VLDB 2015) |
| Objeto no lugar errado | TF consultado no carimbo da foto | tf2 |

O limite de idade sai da física: `max_age_s = tolerancia_posicao_m / v_max_m_s`.
Com 0,20 m de tolerância e 0,30 m/s, o limite é cerca de 0,67 s.

## Requisitos (EARS)

| ID | Requisito | Verificação |
|---|---|---|
| 009a-R01 | O sistema deve aceitar o argumento de launch `offload:=true\|false`. Com `false`, só o detector local roda, e o comportamento é igual ao da spec 004. | launch_testing |
| 009a-R02 | Todo detector, local ou na nuvem, deve publicar `Detection2DArray` com o `header` idêntico ao da imagem de entrada, e deve publicar um array vazio quando não detectar nada. | unitário |
| 009a-R03 | Enquanto `offload:=true`, o robô deve enviar à nuvem apenas a imagem RGB, comprimida, a `image_rate_hz` (padrão 2 Hz). A profundidade não deve sair do robô. | contrato |
| 009a-R04 | O detector local deve ficar carregado e processando desde a subida (reserva quente). | launch_testing |
| 009a-R05 | Para cada resposta recebida, o seletor deve calcular `idade = agora − header.stamp` com o relógio do robô e descartar a resposta se `idade > max_age_s`, incrementando `dropped_stale`. | unitário |
| 009a-R06 | O seletor deve repassar uma resposta somente se o seu `header.stamp` for estritamente maior que a marca d'água, atualizar a marca ao repassar e, caso contrário, descartar e incrementar `dropped_watermark`. | unitário |
| 009a-R07 | Enquanto estiver em CLOSED, o seletor deve repassar apenas respostas da nuvem. Enquanto estiver em OPEN ou HALF_OPEN, apenas respostas locais. | unitário |
| 009a-R08 | Quando estiver em CLOSED e nenhuma resposta da nuvem com `idade ≤ open_threshold_s` (padrão 1,5 s) chegar dentro de `open_threshold_s`, o seletor deve passar a OPEN e emitir uma `Notification` WARN `CLOUD_DETECTOR_DOWN`. | unitário + falha injetada |
| 009a-R09 | Quando estiver em OPEN e chegar uma resposta da nuvem com `idade ≤ close_threshold_s` (padrão 0,5 s), o seletor deve passar a HALF_OPEN. | unitário |
| 009a-R10 | Quando estiver em HALF_OPEN e todas as respostas da nuvem permanecerem com `idade ≤ close_threshold_s` por `stable_window_s` (padrão 3 s), o seletor deve passar a CLOSED e emitir `Notification` INFO `CLOUD_DETECTOR_RECOVERED`. Se alguma resposta violar o limite, deve voltar a OPEN. | unitário |
| 009a-R11 | Enquanto estiver em OPEN ou HALF_OPEN, o robô deve continuar enviando imagens à nuvem na mesma taxa, para servir de sonda. | contrato |
| 009a-R12 | O `projector` deve consultar o TF no `header.stamp` da detecção, com tempo limite `tf_timeout_s` (padrão 0,1 s). Se a transformação não existir, deve descartar e incrementar `dropped_tf`. Nunca deve usar o instante de chegada. | unitário |
| 009a-R13 | O `projector` deve publicar `/detections_markers` no formato que o nó semântico do 2025S2 já consome. | contrato + teste com o nó dele |
| 009a-R14 | O seletor deve publicar `OffloadStatus` a 1 Hz. | contrato |
| 009a-R15 | Quando o Pod do detector da nuvem for apagado durante a limpeza, o seletor deve chegar a OPEN em até `open_threshold_s + 0,5 s`, e o modo do orquestrador não deve mudar. | falha injetada |
| 009a-R16 | Quando o Pod voltar, o seletor deve chegar a CLOSED, e nenhuma detecção com `header.stamp` repetido ou menor que a marca d'água deve aparecer em `/myRobot/perception/detections` em todo o teste. | falha injetada |
| 009a-R17 | O detector da nuvem deve rodar como Deployment no K3s com `livenessProbe`, a partir de uma imagem por distribuição construída pela CI, e a infraestrutura deve ser declarada em Terraform (módulo `local`). | CI + `terraform plan` |
| 009a-R18 | Os requisitos R01 a R16 devem valer em Jazzy e em Lyrical. | CI (matriz) |

## Critério de pronto

- Testes unitários do seletor (R05 a R10) e do `projector` (R12) passando nas duas distribuições.
- Teste de falha injetada (R15, R16) executado e medido: tempo até OPEN, tempo até CLOSED e contadores de descarte.
  Esses números vão para o relatório.
- Q-06 decidida, com a medição que justifica a escolha.
