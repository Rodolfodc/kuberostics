# Constituição do projeto: robô aspirador autônomo (IA368II 2026S2)

Princípios que valem para toda especificação, plano, tarefa e linha de código deste repositório.
Uma especificação que contradiga um artigo é inválida até que o artigo seja emendado explicitamente.

## Artigo I. O contrato vem antes do código

1. Toda interface ROS 2 (tópico, serviço, ação, parâmetro público) existe primeiro em
   `specs/000-sistema/contracts/`, com nome, tipo, QoS (*Quality of Service*), quem publica e quem consome.
2. Os tipos novos são definidos em arquivos `.msg`, `.srv` e `.action` do pacote `aspirador_interfaces`.
   Esse pacote não contém implementação.
3. Mudar um contrato exige mudar a especificação no mesmo commit.

## Artigo II. O código de terceiros não é alterado

1. `lucianovk/IA368II` (pacotes `myrobot_bringup`, `myrobot_interfaces` e `my_robot_pkg`) é uma dependência
   somente leitura, fixada por commit em `deps.repos`.
2. A adaptação usa só estes mecanismos, nesta ordem de preferência:
   1. submodos publicados por um nó próprio;
   2. remapeamento de nomes no launch;
   3. substituição de um nó por outro de mesmo nome que cumpra o mesmo contrato;
   4. herança pontual.
3. É proibido herdar ou substituir `BehaviourManager` (o orquestrador) e estender o enum `Mode`.
   O orquestrador continua publicando exatamente CLEANING, AUTODOCKING, CHARGING, CHARGED e IDLE.

## Artigo III. Todo módulo é plugável

1. Cada módulo é um pacote ROS 2 com launch próprio. Ele sobe sozinho, alimentado por stubs ou rosbag.
2. Nós novos são `LifecycleNode`, gerenciados por um lifecycle manager do Nav2.
3. **Atuador** é o nó que move o robô. Ele publica apenas na sua entrada do `twist_mux`.
4. **Serviço** é o nó que percebe, calcula ou mostra. Ele nunca publica comando de movimento.
5. Duas implementações da mesma interface devem poder trocar de lugar só pelo launch.

## Artigo IV. O sistema sempre roda

1. A primeira entrega é o esqueleto andante (*walking skeleton*, Cockburn): todos os módulos presentes como stubs,
   integrados de ponta a ponta.
2. Um stub só é removido quando a implementação real passa nos mesmos testes de contrato.

## Artigo V. Teste de contrato é obrigatório

1. Cada interface do contrato tem teste automático de existência, tipo e QoS.
2. Cada módulo tem ao menos um teste com `launch_testing`.
3. A integração contínua roda `colcon build` e `colcon test` sem simulador.
   Os testes com CoppeliaSim rodam localmente e são marcados como tal.

## Artigo VI. Segurança de movimento

1. Somente o `twist_mux` publica em `/myRobot/cmd_vel`.
2. Existe um lock de parada de prioridade máxima.
3. Todo comportamento de recuperação tem tempo limite e fallback determinístico.
   Quando ambos falham, o robô para e notifica.

## Artigo VII. A nuvem é opcional

1. O robô limpa a casa sem rede.
2. Os componentes de nuvem (ponte MQTT, serviços em K3s) sobem por um lifecycle manager separado.
3. Pesos de política novos só são carregados com o orquestrador no modo CHARGED.

## Artigo VIII. Rastreabilidade

1. Cada requisito tem um identificador (por exemplo, `001-R04`), escrito em notação EARS
   (*Easy Approach to Requirements Syntax*, Mavin et al., 2009).
2. Cada tarefa cita os requisitos que atende, e cada teste cita o requisito que verifica.
3. As mensagens de commit citam o identificador da tarefa.

## Artigo IX. Simplicidade e prazo

1. Todo módulo chega primeiro em versão mínima funcional e só depois ganha profundidade.
2. Prazo final: 4 de dezembro de 2026, 23:59.
3. Ordem de corte em caso de atraso: (1) agente LLM; (2) retreino na nuvem; (3) política RL, mantendo a fuga por roteiro fixo.
   Os seis módulos do enunciado não são cortados.

## Artigo X. Duas distribuições: Jazzy e Lyrical

1. Todo pacote deste repositório compila, passa nos testes e roda em ROS 2 Jazzy (Ubuntu 24.04, Python 3.12)
   e em ROS 2 Lyrical Luth (Ubuntu 26.04, Python 3.14).
2. A integração contínua roda a matriz completa. Um teste que passa numa distribuição e falha na outra conta como falha.
3. Nenhum comportamento depende de valor padrão de parâmetro de terceiros que mude entre distribuições.
   Esses parâmetros são sempre declarados explicitamente na configuração. Exemplo: `use_stamped` do `twist_mux`
   e `enable_stamped_cmd_vel` do Nav2.
4. Código específico de uma distribuição só é aceito atrás de um teste de `ROS_DISTRO`, com justificativa na spec.
5. Cada execução usa uma única distribuição. Um grafo que mistura nós Jazzy e Lyrical não é suportado.

## Pilha técnica

- ROS 2 Jazzy e ROS 2 Lyrical Luth, Python (`rclpy`), cada uma em contêiner próprio.
- CoppeliaSim com ZeroMQ Remote API, no hospedeiro e independente da distribuição ROS.
- Nav2, SLAM Toolbox, `twist_mux`.
- Gymnasium e Stable-Baselines3.
- K3s, Mosquitto, MinIO, DynamoDB Local, Terraform.

**Versão:** 1.1 · 2026-11-06. Artigo X incluído: suporte a Jazzy e Lyrical.
