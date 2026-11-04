# 001: Tarefas

`[P]` = pode ser feita em paralelo com outras `[P]` da mesma fase. Cada tarefa cita os requisitos que atende.
Uma tarefa só está pronta quando o teste que verifica o requisito passa.

## Fase 0: preparação

- [ ] **T001** Criar `deps.repos` com `lucianovk/IA368II` no commit `51a5fa2c…`. Escrever `scripts/setup.sh`, que roda `vcs import` e `rosdep install`. (D-10)
- [ ] **T002** [P] Criar `aspirador_interfaces` com `Submode`, `StuckEvent`, `Notification`, `Escape` e `CoverRoom`, copiados de `contracts/interfaces.md`. Verificar com `colcon build` e `ros2 interface show`. (001-R04)
- [ ] **T003** [P] Criar `.github/workflows/ci.yaml` em contêiner `ros:jazzy`: setup, `colcon build`, `colcon test`. (001-R15)
- [ ] **T004** [P] Escrever `tools/sim_bench/bench.py`, rodar na cena *House* e registrar D-11. Responder Q-01 no caminho. (001-R14)

## Fase 1: testes primeiro

- [ ] **T005** Escrever `aspirador_contract_tests` lendo `topics.yaml`. O teste deve falhar enquanto o launch não existir. (001-R03, 001-R04, 001-R12)
- [ ] **T006** [P] Escrever os testes pytest de `SubmodeMachine`, uma linha da tabela de transições por caso. (001-R05 a 001-R09)

## Fase 2: implementação

- [ ] **T007** Criar `aspirador_bringup/config/twist_mux.yaml` e confirmar o tipo da saída do `twist_mux` no Jazzy (Q-02). (001-R10, 001-R11)
- [ ] **T008** Implementar `SubmodeMachine` até a T006 passar. (001-R05 a 001-R09)
- [ ] **T009** Implementar o `LifecycleNode` `supervisor`: assina o modo, publica o submodo a 5 Hz, cliente das ações, lock e notificações. (001-R05 a 001-R09)
- [ ] **T010** [P] Stub `coverage_server`: servidor de `CoverRoom` que publica um `Twist` constante em `mux/coverage` enquanto o objetivo está ativo. (001-R06)
- [ ] **T011** [P] Stub `escape_server` com o parâmetro `outcome` (`succeed`, `fail`, `hang`): dá ré e gira por 2 s em `mux/escape`. (001-R07, 001-R16)
- [ ] **T012** [P] Stub `stuck_detector`: publica um `StuckEvent` ao receber a chamada do serviço `~/trigger`. A detecção real fica na spec 005. (001-R07)
- [ ] **T013** Criar `skeleton.launch.py`, com:
  - os seis nós do Luciano e o lifecycle manager dele, com os remapeamentos de D-04 e D-05;
  - o `lifecycle_manager_core`;
  - o `twist_mux`;
  - a ponte de rodas do 2025S2, com o remapeamento de D-06;
  - `teleop_twist_keyboard` com saída em `mux/teleop`.

  (001-R01, 001-R02, 001-R12, 001-R13)

## Fase 3: verificação

- [ ] **T014** Escrever testes `launch_testing` para os cenários 1 a 4, usando o stub de fuga em cada `outcome`. (001-R06 a 001-R11)
- [ ] **T015** Rodar no CoppeliaSim e gravar o vídeo curto dos cenários 1 a 4. (001-R13)
- [ ] **T016** Revisar a spec 000: fechar Q-01 e Q-02 e registrar D-11.
