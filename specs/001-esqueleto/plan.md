# 001: Plano técnico

## Estrutura do repositório

```
aspirador-ia368ii/
├── .specify/memory/constitution.md
├── specs/
│   ├── 000-sistema/{spec.md, contracts/}
│   └── 001-esqueleto/{spec.md, plan.md, tasks.md}
├── deps.repos                       # lucianovk/IA368II fixado por commit (D-10)
├── src/
│   ├── aspirador_interfaces/        # .msg e .action, sem código (Art. I)
│   ├── aspirador_bringup/           # launch, twist_mux.yaml, lifecycle_core.yaml
│   ├── aspirador_supervisor/        # máquina de submodos
│   ├── aspirador_stubs/             # coverage_server, escape_server, stuck_detector, teleop_fake
│   └── aspirador_contract_tests/    # testes gerados a partir de topics.yaml
├── tools/sim_bench/                 # medição de passos/s
└── .github/workflows/ci.yaml
```

O código do Luciano entra por `vcs import src/deps < deps.repos` e fica em `src/deps/`, que é ignorado pelo git.

## Dependências entre nós

```
behaviour_server (Luciano) ──mode──▶ supervisor ──submode──▶ (observação)
                                        │  ├──action──▶ coverage_server ──▶ mux/coverage ─┐
stuck_detector ──StuckEvent──▶──────────┘  ├──action──▶ escape_server   ──▶ mux/escape   ─┤
                                           └──Bool────▶ mux/lock_stop                      ├─▶ twist_mux ─▶ /myRobot/cmd_vel ─▶ ponte de rodas (2025S2) ─▶ CoppeliaSim
autodocking_server (Luciano) ──remap──▶ mux/docking ─────────────────────────────────────────┤
teleop ──────────────────────────────▶ mux/teleop ──────────────────────────────────────────┘
cleaning_server (Luciano) ──remap──▶ /myRobot/legacy/cleaning_cmd_vel (sem assinante, D-04)
```

## Supervisor: máquina de estados

Entradas: modo do orquestrador, `StuckEvent`, resultado das ações.
Estado interno: `submode` e `resume_submode`, que guarda para onde voltar depois da fuga.

| Estado atual | Evento | Ação | Próximo |
|---|---|---|---|
| qualquer | modo ∉ {CLEANING, AUTODOCKING} | cancela ações em curso | NONE |
| NONE | modo = CLEANING | envia `cover_room` | COVERING |
| COVERING | `StuckEvent` | cancela `cover_room`; `resume_submode = COVERING`; envia `escape` | ESCAPING |
| NONE com modo AUTODOCKING | `StuckEvent` | `resume_submode = NONE`; envia `escape` | ESCAPING |
| ESCAPING | `escape` com sucesso | reenvia a ação do `resume_submode`, se houver | `resume_submode` |
| ESCAPING | `escape` falhou ou estourou o tempo | `lock_stop = true`; `Notification` ERROR | ESCAPE_FAILED |
| ESCAPE_FAILED | serviço `~/reset` (do usuário) | `lock_stop = false` | NONE |

EXPLORING e GOTO_ROOM existem no tipo, mas só entram em uso nas specs 003 e 005.

A lógica fica numa classe Python pura (`SubmodeMachine`), sem ROS, testada com pytest.
O `LifecycleNode` só faz a ligação entre essa classe e os tópicos e ações. Assim a regra de transição é testável sem simulador.

## twist_mux

```yaml
twist_mux:
  ros__parameters:
    topics:
      teleop:   {topic: /myRobot/mux/teleop,   timeout: 0.5, priority: 100}
      escape:   {topic: /myRobot/mux/escape,   timeout: 0.5, priority: 80}
      coverage: {topic: /myRobot/mux/coverage, timeout: 0.5, priority: 50}
      docking:  {topic: /myRobot/mux/docking,  timeout: 0.5, priority: 40}
    locks:
      stop:     {topic: /myRobot/mux/lock_stop, timeout: 0.0, priority: 255}
```

Saída `cmd_vel_out` remapeada para `/myRobot/cmd_vel`. A Q-02 (tipo da saída no Jazzy) precisa ser confirmada na T007.

## Teste de contrato

`aspirador_contract_tests` lê `specs/000-sistema/contracts/topics.yaml`, sobe o launch do esqueleto e, para
cada interface, consulta `get_publishers_info_by_topic` e `get_subscriptions_info_by_topic`. O teste falha se:

- o tópico não existe;
- o tipo difere do declarado;
- o QoS difere do declarado;
- um tópico com `max_publishers` tem publicadores demais;
- um tópico `sink` tem assinantes.

Esse teste é a forma executável da spec 000.

## Medição de passos/s (D-11)

`tools/sim_bench/bench.py` conecta pela ZeroMQ Remote API, liga `sim.setStepping(True)`, executa N chamadas de
`sim.step()` com o robô andando em linha reta e imprime passos/s. Deve rodar com o CoppeliaSim iniciado sem
interface gráfica (`-h`).

Regra de decisão da spec 008:
- **200 passos/s ou mais:** treino direto no CoppeliaSim;
- **menos que isso:** treino no ambiente 2D, com validação no CoppeliaSim.

## Riscos

- Nomes de objetos da cena *House* diferentes da cena do 2025S2 (Q-01). Mitigação: os nomes viram parâmetros na ponte de rodas, via remapeamento ou parâmetros do nó do 2025S2.
- Ordem de subida entre o lifecycle manager do Luciano e o `lifecycle_manager_core`. Mitigação: os nós novos não dependem dos dele para ativar, só para receber o modo.
