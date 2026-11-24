# Proposed amendment to Article III.2 (PENDING the project owner's edit)

Status: **not applied.** The constitution is edited only by the project owner (the agents and the protection hook cannot
write `.specify/memory/constitution.md`). Decision basis: D-26 / Q-17 (project owner, 2026-11-14): Article III.2 is not
absolute law; a node is a lifecycle node when a manager must control its states.

Current text:

> 2. New nodes are `LifecycleNode`s, managed by a Nav2 lifecycle manager.

Proposed text:

> 2. New nodes are `LifecycleNode`s managed by a Nav2 lifecycle manager when they hold resources or state that a manager
>    must configure, activate, deactivate or clean up (drivers, publishers that must stop, timers, action servers,
>    Nav2 servers). A node that only orchestrates the lifecycle managers, is one-shot bootstrap glue, or is a tool with no
>    controllable state may be a plain node. Every plain node is listed with its reason in `docs/lifecycle-managers.md`.

Effect: removes the conflict that blocked 005-R35 to R38 and T521; no code or requirement depends on the exact wording.
After the user applies it, delete this file or mark it applied.
