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

## Diagramas

`docs/arquitetura.md` reúne os diagramas de arquitetura e os fluxos de comunicação em Mermaid, que o GitHub renderiza direto. As fontes ficam em `docs/diagramas/*.mmd` e as imagens geradas em `docs/diagramas/render/`.

## Documentação

O PDF `docs/diagramas_arquitetura_aspirador.pdf` é uma exportação estática dos mesmos diagramas.

## Estado

| Spec | Status |
|---|---|
| 000-sistema | rascunho para revisão |
| 001-esqueleto | rascunho para revisão |
| 009a-externalizacao | rascunho para revisão |

## Licença

Apache License 2.0 (ver `LICENSE`). Você pode usar, modificar e contribuir, desde que mantenha o aviso de
copyright e o arquivo `NOTICE`, que atribuem o trabalho a Rodolfo Dalla Costa. Material de terceiros
está listado no `NOTICE`.
