# Kuberostics: autonomous vacuum robot (IA368II 2026S2)

Final project of the course IA368II (FEEC/UNICAMP). A vacuum robot in the *House* scene of CoppeliaSim,
on ROS 2 Jazzy and ROS 2 Lyrical, integrated with the *Robot LifeCycle manager* of the `IA368II` repository
without changing its code, with an escape policy learned by reinforcement learning and a
cloud service emulated on K3s.

## How the repository is developed

Spec-driven development. The reading order is:

1. `.specify/memory/constitution.md`: principles that no spec may violate.
2. `specs/000-system/`: system boundaries, inherited contract, decisions and the executable contract (`contracts/topics.yaml`).
3. `specs/NNN-*/`: one folder per feature, with three files:
   - `spec.md`: what and why, with EARS requirements;
   - `plan.md`: how;
   - `tasks.md`: steps traceable to the requirements.

The layout follows GitHub Spec Kit, so its `/specify`, `/plan` and `/tasks` commands work here.
Using Spec Kit is optional.

## Diagrams

`docs/architecture.md` gathers the architecture diagrams and the communication flows in Mermaid, which GitHub renders directly. The sources live in `docs/diagrams/*.mmd` and the generated SVG images in `docs/diagrams/render/`.

## Status

| Spec | Status |
|---|---|
| 000-system | draft for review |
| 001-skeleton | draft for review |
| 009a-offload | draft for review |

## Installing on another machine

Ubuntu 24.04 or 26.04.

**Prerequisites (install them yourself, the script does not):**
- **ROS 2: required.** Lyrical (Ubuntu 26.04) or Jazzy (Ubuntu 24.04), from the official guide at https://docs.ros.org. Nothing builds without it. Jazzy can also run through Docker (`--pull-jazzy`), but Lyrical must be on the host.
- **llama.cpp: optional.** A local server compatible with the OpenAI API (`http://127.0.0.1:11434/v1`) that the agents use for cheap operational tasks (summarizing logs, drafting boilerplate). The project works without it. Install it by building https://github.com/ggml-org/llama.cpp and downloading a `.gguf` model.

Then:

```bash
git clone https://github.com/Rodolfodc/kuberostics.git && cd kuberostics
scripts/setup_env.sh            # installs what is missing (asks for sudo) and verifies; safe to rerun
scripts/setup_env.sh --check    # only verifies, installs nothing
```

It installs apt packages, Docker, kubectl, k3d, terraform, the firewall for ports 23000-23099, CoppeliaSim 4.10.0,
the `.venv` (`requirements.txt`) and the third-party code (`deps.repos`). The *House* scene (`scene_house.ttt`) lives
outside the repository: put it one folder above the project root. ROS 2 and llama.cpp (optional) are not installed by it.

## License

Apache License 2.0 (see `LICENSE`). You may use, modify and contribute, as long as you keep the copyright notice
and the `NOTICE` file, which credit the work to Rodolfo Dalla Costa. Third-party material is listed in `NOTICE`.
