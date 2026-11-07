# Architecture diagrams

Source of truth: the `.mmd` files in `docs/diagrams/`. The images in `docs/diagrams/render/` are generated with:

```bash
for f in docs/diagrams/*.mmd; do mmdc -i "$f" -o "docs/diagrams/render/$(basename "${f%.mmd}").svg"; done
```

## 1. Overview

System layers: CoppeliaSim on the host, the ROS 2 container (Jazzy or Lyrical) and the K3s cluster. Grey: IA368II code, unchanged. Green: new. Dashed: optional.

```mermaid
flowchart TB
  subgraph K3S["Local K3s cluster · provisioned by Terraform"]
    ZC["Cluster Zenoh"]
    DET["detector_cloud<br/>yolo11x · Deployment"]
    MQ["Mosquitto"]
    DATA["MinIO · DynamoDB Local"]
    TRAINING["Training jobs + digital twin"]
    DASH["Dashboard"]
    LLM["LLM agent (optional)"]
  end

  subgraph ROBOT["ROS 2 container · Jazzy or Lyrical · namespace /myRobot"]
    SRV["Services<br/>schedule · interface · notifications"]
    ORCH["IA368II orchestrator (unchanged)<br/>CLEANING · AUTODOCKING · CHARGING · CHARGED · IDLE"]
    SUP["Submode supervisor<br/>COVERING · ESCAPING · ESCAPE_FAILED ..."]
    MOTION["Actuators + twist_mux<br/>sole publisher of /myRobot/cmd_vel"]
    BRIDGES["ZeroMQ bridges (2025S2)<br/>laser · camera · odometry · wheels"]
    PERC["Perception<br/>SLAM · rooms · detection · selector · projector"]
    ZR["Transport to the cloud<br/>rmw_zenoh or Zenoh bridge (Q-06)"]
    MQB["MQTT bridge<br/>(optional, at the dock only)"]
  end

  subgraph HOST["Host"]
    SIM["CoppeliaSim<br/>House scene"]
  end

  SRV -->|"start / stop"| ORCH
  ORCH -->|"mode"| SUP
  SUP -->|"actions and lock"| MOTION
  MOTION -->|"/myRobot/cmd_vel"| BRIDGES
  BRIDGES -->|"sensors"| PERC
  PERC -->|"stuck"| SUP
  BRIDGES <-->|"ZeroMQ :23000"| SIM
  PERC <--> ZR
  ZR <-->|"TCP 7447"| ZC
  ZC <--> DET
  MQB <-->|"MQTT"| MQ
  MQ --> DATA --> TRAINING --> DASH
  LLM -.->|"traps"| TRAINING

  classDef ia368ii fill:#ECEAE4,stroke:#8A8880,color:#2C2C2A
  classDef added fill:#E1F5EE,stroke:#0F6E56,color:#04342C
  classDef optional fill:#FFFFFF,stroke:#0F6E56,stroke-dasharray:4 3,color:#04342C
  class ORCH,BRIDGES ia368ii
  class SUP,MOTION,PERC,SRV,ZR,ZC,DET,MQ,DATA,TRAINING,DASH added
  class MQB,LLM optional
```

## 2. Control and motion

The three lifecycle managers, the orchestrator mode, the supervisor actions and the `twist_mux` inputs with priority. Orange: reuse from the 2025S2 project.

```mermaid
flowchart TB
  subgraph LMIA["IA368II lifecycle_manager (unchanged)"]
    BEH["behaviour_server"]
    MYR["myrobot<br/>battery and base (simulated)"]
    CLE["cleaning_server<br/>stub"]
    DOC["autodocking_server"]
    CHG["charging_server · charged_server"]
  end

  subgraph LMCORE["lifecycle_manager_core"]
    SUP["supervisor"]
    STK["stuck_detector"]
    ESC["escape_server<br/>scripted or RL"]
    COV["coverage_server"]
    STOPG["stop_guard<br/>zero velocity while locked"]
  end

  subgraph LMOPT["lifecycle_manager_optional"]
    MQB["MQTT bridge"]
  end

  NAV["Nav2 controller<br/>enable_stamped_cmd_vel: false"]
  TEL["teleop_twist_keyboard"]
  SCHED["schedule (module 4)"]
  HMI["interface (module 6)"]
  MUX["twist_mux<br/>use_stamped: false"]
  WHEEL["wheel bridge (2025S2)<br/>/cmd_vel → /myRobot/cmd_vel"]
  SIM["CoppeliaSim"]
  SINK(["/myRobot/legacy/cleaning_cmd_vel<br/>no subscriber (D-04)"])

  BEH -->|"/myRobot/behaviour/mode"| SUP
  BEH -->|"mode"| MYR
  BEH -->|"mode"| CLE
  BEH -->|"mode"| DOC
  MYR -->|"battery_state · charging_base"| BEH
  SCHED -->|"/myRobot/start_stop"| BEH
  STK -->|"/myRobot/stuck"| SUP
  SUP -->|"cover_room action"| COV
  SUP -->|"escape action"| ESC
  SUP -->|"/myRobot/notifications"| HMI

  TEL -->|"mux/teleop · 100"| MUX
  ESC -->|"mux/escape · 80"| MUX
  NAV -->|"mux/nav · 60"| MUX
  COV -->|"mux/coverage · 50"| MUX
  DOC -->|"mux/docking · 40 (remap D-05)"| MUX
  SUP -->|"mux/lock_stop · lock 200"| MUX
  SUP -->|"lock_stop"| STOPG
  STOPG -->|"mux/stop · 250 (D-22)"| MUX
  CLE -->|"remap D-04"| SINK

  MUX -->|"/myRobot/cmd_vel (Twist)"| WHEEL
  WHEEL -->|"ZeroMQ"| SIM

  classDef ia368ii fill:#ECEAE4,stroke:#8A8880,color:#2C2C2A
  classDef added fill:#E1F5EE,stroke:#0F6E56,color:#04342C
  classDef reused fill:#FAEEDA,stroke:#BA7517,color:#412402
  classDef optional fill:#FFFFFF,stroke:#0F6E56,stroke-dasharray:4 3,color:#04342C
  class BEH,MYR,CLE,DOC,CHG ia368ii
  class SUP,STK,ESC,COV,STOPG,SCHED,HMI,MUX,NAV,TEL added
  class WHEEL reused
  class MQB optional
```

## 3. Perception and offload

Only the compressed RGB image goes up to the cloud. Depth and TF stay on the robot, and the projector queries both at the photo timestamp (D-16).

```mermaid
flowchart LR
  subgraph CLOUD["K3s"]
    DC["detector_cloud<br/>yolo11x · Deployment + livenessProbe"]
  end

  subgraph ROBOT["Robot"]
    VIS["my_robot_vision_node (2025S2)<br/>/camera · /camera_depth"]
    THR["image_throttle<br/>compressed RGB · 2 Hz"]
    DL["detector_local<br/>yolo11n · hot standby"]
    SEL["detection_selector<br/>circuit breaker · age · watermark"]
    CACHE[("depth cache<br/>by timestamp")]
    TF[("TF")]
    PROJ["projector<br/>2D + depth + TF<br/>at the photo timestamp"]
    SEM["semantic segmentation (2025S2)<br/>/semantic_map"]
    subgraph MAPPING["Mapping (module 2)"]
      SCAN["my_robot_scan_node (2025S2)<br/>/scan"]
      SLAM["SLAM Toolbox<br/>/map"]
      TOPO["topological segmentation (2025S2)<br/>/topologic_map"]
    end
  end

  VIS -->|"/camera"| THR
  THR -->|"rgb_throttled/compressed<br/>via Zenoh"| DC
  DC -->|"detections/cloud<br/>header = photo header"| SEL
  VIS -->|"/camera"| DL
  DL -->|"detections/local"| SEL
  SEL -->|"/myRobot/perception/detections"| PROJ
  VIS -->|"/camera_depth"| CACHE
  CACHE --> PROJ
  TF --> PROJ
  PROJ -->|"/detections_markers"| SEM
  SCAN --> SLAM --> TOPO -->|"/topologic_map"| SEM
  SEL -->|"offload_status<br/>notifications"| OUT(["interface and logs"])

  classDef reused fill:#FAEEDA,stroke:#BA7517,color:#412402
  classDef added fill:#E1F5EE,stroke:#0F6E56,color:#04342C
  class VIS,SCAN,TOPO,SEM reused
  class THR,DL,SEL,PROJ,DC,SLAM added
```

## 4. Cloud and asynchronous cycle (spec 009b, planned)

Stuck events go up at the dock, training produces a candidate policy, the digital twin evaluates it, the dashboard approves it and the robot downloads the weights with a conditional request.

```mermaid
flowchart TB
  subgraph ROBOT["Robot"]
    STK["stuck_detector<br/>StuckEvent with laser and pose"]
    LOG[("local stuck<br/>log")]
    MQB["MQTT bridge<br/>only in CHARGED"]
    ESC["escape_rl<br/>loads weights only in CHARGED"]
    HMI["interface<br/>user complaint"]
  end

  subgraph K3S["Local K3s (Terraform local module)"]
    MQ["Mosquitto"]
    ING["ingestion"]
    RAW[("MinIO<br/>raw stuck events")]
    IDX[("DynamoDB Local<br/>index and history")]
    LLM["LLM agent (optional)<br/>complaint → trap as validated JSON"]
    GEN["trap generator"]
    JOB["Training Job<br/>Gymnasium + Stable-Baselines3"]
    TWIN["digital twin<br/>headless CoppeliaSim · unseen traps"]
    DASH["dashboard<br/>human approval"]
    W[("MinIO<br/>versioned weights (ETag)")]
  end

  subgraph AWS["Same design on AWS (Terraform aws module)"]
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
  RAW --> GEN --> JOB --> TWIN --> DASH --> W
  W -->|"GET with If-None-Match<br/>200 with new weights or 304"| MQB
  MQB --> ESC

  MQ ~~~ A1
  RAW ~~~ A2
  IDX ~~~ A3
  JOB ~~~ A4
  LLM ~~~ A5

  classDef added fill:#E1F5EE,stroke:#0F6E56,color:#04342C
  classDef optional fill:#FFFFFF,stroke:#0F6E56,stroke-dasharray:4 3,color:#04342C
  classDef aws fill:#F1EFE8,stroke:#888780,color:#2C2C2A
  class STK,MQB,ESC,HMI,MQ,ING,GEN,JOB,TWIN,DASH added
  class LLM optional
  class A1,A2,A3,A4,A5 aws
```

## 5. Flow: normal offload path

Timestamp echo: the photo header goes through the detector and comes back, and the robot measures the age with its own clock.

```mermaid
sequenceDiagram
  autonumber
  participant CAM as Camera (vision_node)
  participant THR as image_throttle
  participant DC as detector_cloud (K3s)
  participant DL as detector_local
  participant SEL as detection_selector
  participant PROJ as projector
  participant SEM as semantic segmentation

  CAM->>THR: /camera (stamp = t0)
  CAM->>DL: /camera (stamp = t0)
  Note over CAM,PROJ: /camera_depth (t0) stays in the projector cache
  THR->>DC: compressed RGB (stamp = t0)
  DL-->>SEL: detections/local (header.stamp = t0)
  Note over SEL: state CLOSED: local response ignored
  DC->>SEL: detections/cloud (header.stamp = t0, echo)
  Note over SEL: age = now − t0 ≤ max_age_s<br/>t0 > watermark → forwards<br/>watermark = t0
  SEL->>PROJ: /myRobot/perception/detections (t0)
  Note over PROJ: depth from cache at t0<br/>TF queried at t0
  PROJ->>SEM: /detections_markers (position on the map)
```

## 6. Flow: circuit breaker states

Hysteresis: opening threshold (1.5 s) different from the closing one (0.5 s held for 3 s).

```mermaid
stateDiagram-v2
  direction LR
  [*] --> CLOSED: offload on
  CLOSED --> OPEN: no cloud response<br/>with age ≤ 1.5 s for 1.5 s<br/>(Notification WARN)
  OPEN --> HALF_OPEN: cloud response<br/>with age ≤ 0.5 s
  HALF_OPEN --> OPEN: cloud response<br/>with age > 0.5 s
  HALF_OPEN --> CLOSED: 3 s in a row with<br/>responses ≤ 0.5 s<br/>(Notification INFO)

  note right of CLOSED: forwards the cloud
  note right of OPEN: forwards the local<br/>images keep going up (probe)
  note right of HALF_OPEN: forwards the local<br/>while the cloud proves stability
```

## 7. Flow: selector decision per message

Filter order: active source, age, watermark. The 10 Hz timer detects cloud silence.

```mermaid
flowchart TB
  IN(["Detection2DArray arrives<br/>(cloud or local)"]) --> CLD{"From the cloud?"}
  CLD -->|yes| HEALTH["Update cloud health:<br/>cloud_age = now − header.stamp"]
  CLD -->|no| SOURCE
  HEALTH --> TRANS["Evaluate circuit breaker transitions<br/>(state table)"]
  TRANS --> SOURCE{"Is the source the one of the current state?<br/>CLOSED → cloud<br/>OPEN or HALF_OPEN → local"}
  SOURCE -->|no| DROP0(["Ignore<br/>(inactive source)"])
  SOURCE -->|yes| AGE{"now − header.stamp<br/>≤ max_age_s?"}
  AGE -->|no| DROP1(["Drop<br/>dropped_stale += 1"])
  AGE -->|yes| MARK{"header.stamp ><br/>watermark?"}
  MARK -->|no| DROP2(["Drop<br/>dropped_watermark += 1"])
  MARK -->|yes| PUB["Publish on<br/>/myRobot/perception/detections"]
  PUB --> UPD(["watermark = header.stamp"])

  TICK(["10 Hz timer"]) --> SIL{"CLOSED and no cloud response<br/>with age ≤ 1.5 s for 1.5 s?"}
  SIL -->|yes| OPENS(["Change to OPEN<br/>Notification WARN"])
  SIL -->|no| STAT(["Publish OffloadStatus at 1 Hz"])
```

## 8. Flow: failure and recovery

The test that kills the Pod: circuit opening, late responses dropped by age and duplicates dropped by the watermark.

```mermaid
sequenceDiagram
  autonumber
  participant K8S as Kubernetes
  participant DC as detector_cloud
  participant SEL as detection_selector
  participant DL as detector_local
  participant HMI as interface

  Note over SEL: CLOSED, forwarding the cloud
  K8S-xDC: Pod deleted (failure test)
  Note over SEL: cloud silence
  SEL->>SEL: 1.5 s without a response ≤ 1.5 s → OPEN
  SEL->>HMI: Notification WARN CLOUD_DETECTOR_DOWN
  DL->>SEL: detections/local (forwarded)
  Note over SEL: the robot keeps cleaning<br/>orchestrator mode does not change
  K8S->>DC: Deployment recreates the Pod
  DC->>SEL: late responses about old photos
  Note over SEL: age > max_age_s → dropped_stale
  DC->>SEL: response with age ≤ 0.5 s
  SEL->>SEL: OPEN → HALF_OPEN
  DC->>SEL: stable responses for 3 s
  SEL->>SEL: HALF_OPEN → CLOSED
  SEL->>HMI: Notification INFO CLOUD_DETECTOR_RECOVERED
  DC->>SEL: response about a photo already covered by the local one
  Note over SEL: stamp ≤ watermark → dropped_watermark<br/>(no duplicate on the map)
```

## 9. Flow: dock synchronization (spec 009b, planned)

Only in CHARGED. Stuck events go up over MQTT QoS 1 and weights come down with If-None-Match (304 when nothing changed).

```mermaid
sequenceDiagram
  autonumber
  participant BEH as behaviour_server (IA368II)
  participant MQB as MQTT bridge
  participant MQ as Mosquitto (K3s)
  participant W as MinIO (weights)
  participant ESC as escape_rl

  BEH->>MQB: /myRobot/behaviour/mode = CHARGED
  Note over MQB: syncs only at the dock
  MQB->>MQ: publishes recorded stuck events (MQTT QoS 1)
  MQ-->>MQB: PUBACK
  MQB->>W: GET weights with If-None-Match: current ETag
  alt weights unchanged
    W-->>MQB: 304 Not Modified
  else new approved weights
    W-->>MQB: 200 + weights + new ETag
    MQB->>ESC: reload weights (still in CHARGED)
    ESC-->>MQB: weights loaded (version recorded)
  end
  Note over BEH,ESC: if the network fails, tries again the next time it is at the dock<br/>the robot never waits for the cloud to clean
```
