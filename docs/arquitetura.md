# Diagramas de arquitetura

Fonte de verdade: os arquivos `.mmd` em `docs/diagramas/`. As imagens em `docs/diagramas/render/` são geradas com:

```bash
for f in docs/diagramas/*.mmd; do mmdc -i "$f" -o "docs/diagramas/render/$(basename "${f%.mmd}").svg"; done
```

## 1. Visão geral

Camadas do sistema: CoppeliaSim no hospedeiro, o contêiner ROS 2 (Jazzy ou Lyrical) e o cluster K3s. Cinza: código do Luciano, sem alteração. Verde: novo. Tracejado: opcional.

```mermaid
flowchart TB
  subgraph K3S["Cluster K3s local · provisionado por Terraform"]
    ZC["Zenoh do cluster"]
    DET["detector_cloud<br/>yolo11x · Deployment"]
    MQ["Mosquitto"]
    DATA["MinIO · DynamoDB Local"]
    TREINO["Jobs de treino + gêmeo digital"]
    PAINEL["Painel"]
    LLM["Agente LLM (opcional)"]
  end

  subgraph ROBO["Contêiner ROS 2 · Jazzy ou Lyrical · namespace /myRobot"]
    SRV["Serviços<br/>agenda · interface · notificações"]
    ORQ["Orquestrador do Luciano (inalterado)<br/>CLEANING · AUTODOCKING · CHARGING · CHARGED · IDLE"]
    SUP["Supervisor de submodos<br/>COVERING · ESCAPING · ESCAPE_FAILED ..."]
    MOV["Atuadores + twist_mux<br/>único publicador de /myRobot/cmd_vel"]
    PONTES["Pontes ZeroMQ (2025S2)<br/>laser · câmera · odometria · rodas"]
    PER["Percepção<br/>SLAM · cômodos · detecção · seletor · projetor"]
    ZR["Transporte para a nuvem<br/>rmw_zenoh ou ponte Zenoh (Q-06)"]
    MQB["Ponte MQTT<br/>(opcional, só na base)"]
  end

  subgraph HOST["Hospedeiro"]
    SIM["CoppeliaSim<br/>cena House"]
  end

  SRV -->|"start / stop"| ORQ
  ORQ -->|"modo"| SUP
  SUP -->|"ações e lock"| MOV
  MOV -->|"/myRobot/cmd_vel"| PONTES
  PONTES -->|"sensores"| PER
  PER -->|"travamento"| SUP
  PONTES <-->|"ZeroMQ :23000"| SIM
  PER <--> ZR
  ZR <-->|"TCP 7447"| ZC
  ZC <--> DET
  MQB <-->|"MQTT"| MQ
  MQ --> DATA --> TREINO --> PAINEL
  LLM -.->|"armadilhas"| TREINO

  classDef luciano fill:#ECEAE4,stroke:#8A8880,color:#2C2C2A
  classDef novo fill:#E1F5EE,stroke:#0F6E56,color:#04342C
  classDef opcional fill:#FFFFFF,stroke:#0F6E56,stroke-dasharray:4 3,color:#04342C
  class ORQ,PONTES luciano
  class SUP,MOV,PER,SRV,ZR,ZC,DET,MQ,DATA,TREINO,PAINEL novo
  class MQB,LLM opcional
```

## 2. Controle e movimento

Os três lifecycle managers, o modo do orquestrador, as ações do supervisor e as entradas do twist_mux com prioridade. Laranja: reuso do projeto 2025S2.

```mermaid
flowchart TB
  subgraph LMLUC["lifecycle_manager do Luciano (inalterado)"]
    BEH["behaviour_server"]
    MYR["myrobot<br/>bateria e base (simulados)"]
    CLE["cleaning_server<br/>stub"]
    DOC["autodocking_server"]
    CHG["charging_server · charged_server"]
  end

  subgraph LMCORE["lifecycle_manager_core"]
    SUP["supervisor"]
    STK["stuck_detector"]
    ESC["escape_server<br/>roteiro fixo ou RL"]
    COV["coverage_server"]
  end

  subgraph LMOPT["lifecycle_manager_optional"]
    MQB["ponte MQTT"]
  end

  NAV["Nav2 controller<br/>enable_stamped_cmd_vel: false"]
  TEL["teleop_twist_keyboard"]
  AGE["agenda (módulo 4)"]
  HMI["interface (módulo 6)"]
  MUX["twist_mux<br/>use_stamped: false"]
  WHEEL["ponte de rodas (2025S2)<br/>/cmd_vel → /myRobot/cmd_vel"]
  SIM["CoppeliaSim"]
  SINK(["/myRobot/legacy/cleaning_cmd_vel<br/>sem assinante (D-04)"])

  BEH -->|"/myRobot/behaviour/mode"| SUP
  BEH -->|"modo"| MYR
  BEH -->|"modo"| CLE
  BEH -->|"modo"| DOC
  MYR -->|"battery_state · charging_base"| BEH
  AGE -->|"/myRobot/start_stop"| BEH
  STK -->|"/myRobot/stuck"| SUP
  SUP -->|"ação cover_room"| COV
  SUP -->|"ação escape"| ESC
  SUP -->|"/myRobot/notifications"| HMI

  TEL -->|"mux/teleop · 100"| MUX
  ESC -->|"mux/escape · 80"| MUX
  NAV -->|"mux/nav · 60"| MUX
  COV -->|"mux/coverage · 50"| MUX
  DOC -->|"mux/docking · 40 (remap D-05)"| MUX
  SUP -->|"mux/lock_stop · 255"| MUX
  CLE -->|"remap D-04"| SINK

  MUX -->|"/myRobot/cmd_vel (Twist)"| WHEEL
  WHEEL -->|"ZeroMQ"| SIM

  classDef luciano fill:#ECEAE4,stroke:#8A8880,color:#2C2C2A
  classDef novo fill:#E1F5EE,stroke:#0F6E56,color:#04342C
  classDef reuso fill:#FAEEDA,stroke:#BA7517,color:#412402
  classDef opcional fill:#FFFFFF,stroke:#0F6E56,stroke-dasharray:4 3,color:#04342C
  class BEH,MYR,CLE,DOC,CHG luciano
  class SUP,STK,ESC,COV,AGE,HMI,MUX,NAV,TEL novo
  class WHEEL reuso
  class MQB opcional
```

## 3. Percepção e offload

Só a imagem RGB comprimida sobe para a nuvem. A profundidade e o TF ficam no robô, e o projector consulta os dois no carimbo da foto (D-16).

```mermaid
flowchart LR
  subgraph NUVEM["K3s"]
    DC["detector_cloud<br/>yolo11x · Deployment + livenessProbe"]
  end

  subgraph ROBO["Robô"]
    VIS["my_robot_vision_node (2025S2)<br/>/camera · /camera_depth"]
    THR["image_throttle<br/>RGB comprimida · 2 Hz"]
    DL["detector_local<br/>yolo11n · reserva quente"]
    SEL["detection_selector<br/>circuit breaker · idade · marca d'água"]
    CACHE[("cache de profundidade<br/>por carimbo")]
    TF[("TF")]
    PROJ["projector<br/>2D + profundidade + TF<br/>no carimbo da foto"]
    SEM["segmentação semântica (2025S2)<br/>/semantic_map"]
    subgraph MAPA["Mapeamento (módulo 2)"]
      SCAN["my_robot_scan_node (2025S2)<br/>/scan"]
      SLAM["SLAM Toolbox<br/>/map"]
      TOPO["segmentação topológica (2025S2)<br/>/topologic_map"]
    end
  end

  VIS -->|"/camera"| THR
  THR -->|"rgb_throttled/compressed<br/>via Zenoh"| DC
  DC -->|"detections/cloud<br/>header = header da foto"| SEL
  VIS -->|"/camera"| DL
  DL -->|"detections/local"| SEL
  SEL -->|"/myRobot/perception/detections"| PROJ
  VIS -->|"/camera_depth"| CACHE
  CACHE --> PROJ
  TF --> PROJ
  PROJ -->|"/detections_markers"| SEM
  SCAN --> SLAM --> TOPO -->|"/topologic_map"| SEM
  SEL -->|"offload_status<br/>notifications"| OUT(["interface e logs"])

  classDef reuso fill:#FAEEDA,stroke:#BA7517,color:#412402
  classDef novo fill:#E1F5EE,stroke:#0F6E56,color:#04342C
  class VIS,SCAN,TOPO,SEM reuso
  class THR,DL,SEL,PROJ,DC,SLAM novo
```

## 4. Nuvem e ciclo assíncrono (spec 009b, planejada)

Travamentos sobem na base, o treino gera uma política candidata, o gêmeo digital avalia, o painel aprova e o robô baixa os pesos com requisição condicional.

```mermaid
flowchart TB
  subgraph ROBO["Robô"]
    STK["stuck_detector<br/>StuckEvent com laser e pose"]
    LOG[("registro local<br/>de travamentos")]
    MQB["ponte MQTT<br/>só em CHARGED"]
    ESC["escape_rl<br/>carrega pesos só em CHARGED"]
    HMI["interface<br/>reclamação do usuário"]
  end

  subgraph K3S["K3s local (módulo Terraform local)"]
    MQ["Mosquitto"]
    ING["ingestão"]
    RAW[("MinIO<br/>travamentos brutos")]
    IDX[("DynamoDB Local<br/>índice e histórico")]
    LLM["agente LLM (opcional)<br/>reclamação → armadilha em JSON validado"]
    GEN["gerador de armadilhas"]
    JOB["Job de treino<br/>Gymnasium + Stable-Baselines3"]
    TWIN["gêmeo digital<br/>CoppeliaSim headless · armadilhas não vistas"]
    PAI["painel<br/>aprovação humana"]
    W[("MinIO<br/>pesos versionados (ETag)")]
  end

  subgraph AWS["Mesmo desenho na AWS (módulo Terraform aws)"]
    A1["IoT Core"]
    A2["S3"]
    A3["DynamoDB"]
    A4["EKS Jobs"]
    A5["Bedrock"]
  end

  STK --> LOG --> MQB
  MQB -->|"MQTT QoS 1"| MQ --> ING
  ING --> RAW
  ING --> IDX
  HMI -.-> LLM
  LLM -.-> GEN
  RAW --> GEN --> JOB --> TWIN --> PAI --> W
  W -->|"GET com If-None-Match<br/>200 com pesos novos ou 304"| MQB
  MQB --> ESC

  MQ ~~~ A1
  RAW ~~~ A2
  IDX ~~~ A3
  JOB ~~~ A4
  LLM ~~~ A5

  classDef novo fill:#E1F5EE,stroke:#0F6E56,color:#04342C
  classDef opcional fill:#FFFFFF,stroke:#0F6E56,stroke-dasharray:4 3,color:#04342C
  classDef aws fill:#F1EFE8,stroke:#888780,color:#2C2C2A
  class STK,MQB,ESC,HMI,MQ,ING,GEN,JOB,TWIN,PAI novo
  class LLM opcional
  class A1,A2,A3,A4,A5 aws
```

## 5. Fluxo: caminho normal do offload

Eco do carimbo: o header da foto atravessa o detector e volta, e o robô mede a idade com o próprio relógio.

```mermaid
sequenceDiagram
  autonumber
  participant CAM as Câmera (vision_node)
  participant THR as image_throttle
  participant DC as detector_cloud (K3s)
  participant DL as detector_local
  participant SEL as detection_selector
  participant PROJ as projector
  participant SEM as segmentação semântica

  CAM->>THR: /camera (stamp = t0)
  CAM->>DL: /camera (stamp = t0)
  Note over CAM,PROJ: /camera_depth (t0) fica no cache do projector
  THR->>DC: RGB comprimida (stamp = t0)
  DL-->>SEL: detections/local (header.stamp = t0)
  Note over SEL: estado CLOSED: resposta local ignorada
  DC->>SEL: detections/cloud (header.stamp = t0, eco)
  Note over SEL: idade = agora − t0 ≤ max_age_s<br/>t0 > marca d'água → repassa<br/>marca d'água = t0
  SEL->>PROJ: /myRobot/perception/detections (t0)
  Note over PROJ: profundidade do cache em t0<br/>TF consultado em t0
  PROJ->>SEM: /detections_markers (posição no mapa)
```

## 6. Fluxo: estados do circuit breaker

Histerese: limiar de abertura (1,5 s) diferente do de fechamento (0,5 s mantido por 3 s).

```mermaid
stateDiagram-v2
  direction LR
  [*] --> CLOSED: offload ligado
  CLOSED --> OPEN: nenhuma resposta da nuvem<br/>com idade ≤ 1,5 s por 1,5 s<br/>(Notification WARN)
  OPEN --> HALF_OPEN: resposta da nuvem<br/>com idade ≤ 0,5 s
  HALF_OPEN --> OPEN: resposta da nuvem<br/>com idade > 0,5 s
  HALF_OPEN --> CLOSED: 3 s seguidos com<br/>respostas ≤ 0,5 s<br/>(Notification INFO)

  note right of CLOSED: repassa a nuvem
  note right of OPEN: repassa o local<br/>imagens continuam subindo (sonda)
  note right of HALF_OPEN: repassa o local<br/>enquanto a nuvem prova estabilidade
```

## 7. Fluxo: decisão do seletor por mensagem

Ordem dos filtros: fonte ativa, idade, marca d'água. O timer a 10 Hz detecta o silêncio da nuvem.

```mermaid
flowchart TB
  IN(["Chega Detection2DArray<br/>(nuvem ou local)"]) --> NUV{"Veio da nuvem?"}
  NUV -->|sim| SAUDE["Atualiza a saúde da nuvem:<br/>idade_nuvem = agora − header.stamp"]
  NUV -->|não| FONTE
  SAUDE --> TRANS["Avalia transições do circuit breaker<br/>(tabela de estados)"]
  TRANS --> FONTE{"A fonte é a do estado atual?<br/>CLOSED → nuvem<br/>OPEN ou HALF_OPEN → local"}
  FONTE -->|não| DESC0(["Ignora<br/>(fonte inativa)"])
  FONTE -->|sim| IDADE{"agora − header.stamp<br/>≤ max_age_s?"}
  IDADE -->|não| DESC1(["Descarta<br/>dropped_stale += 1"])
  IDADE -->|sim| MARCA{"header.stamp ><br/>marca d'água?"}
  MARCA -->|não| DESC2(["Descarta<br/>dropped_watermark += 1"])
  MARCA -->|sim| PUB["Publica em<br/>/myRobot/perception/detections"]
  PUB --> ATU(["marca d'água = header.stamp"])

  TICK(["Timer a 10 Hz"]) --> SIL{"CLOSED e nenhuma resposta da nuvem<br/>com idade ≤ 1,5 s há 1,5 s?"}
  SIL -->|sim| ABRE(["Passa a OPEN<br/>Notification WARN"])
  SIL -->|não| STAT(["Publica OffloadStatus a 1 Hz"])
```

## 8. Fluxo: falha e recuperação

O teste que derruba o Pod: abertura do circuito, respostas atrasadas descartadas por idade e duplicatas descartadas pela marca d'água.

```mermaid
sequenceDiagram
  autonumber
  participant K8S as Kubernetes
  participant DC as detector_cloud
  participant SEL as detection_selector
  participant DL as detector_local
  participant HMI as interface

  Note over SEL: CLOSED, repassando a nuvem
  K8S-xDC: Pod apagado (teste de falha)
  Note over SEL: silêncio da nuvem
  SEL->>SEL: 1,5 s sem resposta ≤ 1,5 s → OPEN
  SEL->>HMI: Notification WARN CLOUD_DETECTOR_DOWN
  DL->>SEL: detections/local (repassadas)
  Note over SEL: o robô continua limpando<br/>modo do orquestrador não muda
  K8S->>DC: Deployment recria o Pod
  DC->>SEL: respostas atrasadas sobre fotos antigas
  Note over SEL: idade > max_age_s → dropped_stale
  DC->>SEL: resposta com idade ≤ 0,5 s
  SEL->>SEL: OPEN → HALF_OPEN
  DC->>SEL: respostas estáveis por 3 s
  SEL->>SEL: HALF_OPEN → CLOSED
  SEL->>HMI: Notification INFO CLOUD_DETECTOR_RECOVERED
  DC->>SEL: resposta sobre foto já coberta pelo local
  Note over SEL: stamp ≤ marca d'água → dropped_watermark<br/>(sem duplicata no mapa)
```

## 9. Fluxo: sincronização na base (spec 009b, planejada)

Só em CHARGED. Travamentos sobem por MQTT QoS 1 e pesos descem com If-None-Match (304 quando nada mudou).

```mermaid
sequenceDiagram
  autonumber
  participant BEH as behaviour_server (Luciano)
  participant MQB as ponte MQTT
  participant MQ as Mosquitto (K3s)
  participant W as MinIO (pesos)
  participant ESC as escape_rl

  BEH->>MQB: /myRobot/behaviour/mode = CHARGED
  Note over MQB: só sincroniza na base
  MQB->>MQ: publica travamentos registrados (MQTT QoS 1)
  MQ-->>MQB: PUBACK
  MQB->>W: GET pesos com If-None-Match: ETag atual
  alt pesos não mudaram
    W-->>MQB: 304 Not Modified
  else há pesos aprovados novos
    W-->>MQB: 200 + pesos + ETag novo
    MQB->>ESC: recarregar pesos (ainda em CHARGED)
    ESC-->>MQB: pesos carregados (versão registrada)
  end
  Note over BEH,ESC: se a rede falhar, tenta na próxima vez em que estiver na base<br/>o robô nunca espera a nuvem para limpar
```
