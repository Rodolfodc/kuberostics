# Robô aspirador autônomo (IA368II 2026S2)

Projeto final da disciplina IA368II (FEEC/UNICAMP). Robô aspirador na cena *House* do CoppeliaSim,
em ROS 2 Jazzy e ROS 2 Lyrical Luth, integrado ao *Robot LifeCycle manager* de Luciano (lucianovk/IA368II) sem alterar o código dele, com
política de fuga aprendida por reforço e serviço em nuvem emulado em K3s.

## Como o repositório é desenvolvido

Desenvolvimento guiado por especificação. A ordem de leitura é:

1. `.specify/memory/constitution.md`: princípios que nenhuma spec pode violar.
2. `specs/000-sistema/`: fronteiras do sistema, contrato herdado, decisões e contrato executável (`contracts/topics.yaml`).
3. `specs/NNN-*/`: uma pasta por funcionalidade, com três arquivos:
   - `spec.md`: o quê e por quê, com requisitos em EARS;
   - `plan.md`: como;
   - `tasks.md`: passos rastreáveis até os requisitos.

O layout segue o do GitHub Spec Kit, então os comandos `/specify`, `/plan` e `/tasks` dele funcionam aqui.
O uso do Spec Kit é opcional.

## Estado

| Spec | Status |
|---|---|
| 000-sistema | rascunho para revisão |
| 001-esqueleto | rascunho para revisão |
| 009a-externalizacao | rascunho para revisão |
