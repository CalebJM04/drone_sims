# Drone awareness mesh — team starting point

This branch is a working software simulation foundation for a four-person college
project. Simulated drones exchange position and velocity through a modeled LoRa
radio network and assess collision risk. The simulator runs on a deterministic
virtual clock with recorded random seeds.

The completed implementation is preserved on the `most-updated` branch at commit
`a63595d`. `main` deliberately leaves three substantial features for teammates.
The removed implementations and historical results are available on that branch.
Git history is preserved.

## Run the foundation

Python 3.11 or newer is required. There are no runtime dependencies.

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e .
make test
make scenario
make quick
```

`make scenario` saves a head-on scenario, including events and position traces, to
`results/head_on.json`. `make quick` generates fresh core verification data and a
short text summary under `results/quick/`. Generated results are ignored by Git.
Use `make verify-ci` for the CI-sized campaign or `make verify` for the larger one.
These checks cover the foundation; they do not establish completed mesh acceptance.

## Team work

| Area | Starting files | Short writeup |
| --- | --- | --- |
| Predictive links and proactive routing | `network_awareness.py`, `routing.py` | [Task 1](docs/TASK_1_ROUTING.md) |
| Independent node processes and mesh orchestration | `mesh_demo.py`, `companion.py`, `radio.py` | [Task 2](docs/TASK_2_NODE_PROCESSES.md) |
| Live topology dashboard and mesh acceptance reporting | `mesh_visualization.py`, scenario traces and node snapshots | [Task 3](docs/TASK_3_DASHBOARD.md) |

Read the [current project state](docs/PROJECT_STATE.md) for what works now.
Each feature has a small entrypoint contract that raises `NotImplementedError`.
The normal CLI exposes only `scenario`, `campaign`, and `verify`. Teammates should
add feature commands and tests when their implementations are ready.

Task 2 can begin with basic flooding. Task 3 can begin with fixture snapshots.
Integrate task 1's link and route data after those contracts are agreed.
Each teammate should branch from reduced `main` and submit their feature through
a pull request.

## Foundation layout

- `src/drone_sims/`: motion, telemetry, radio model, state tracking, collision
  assessment, simulated avoidance, core verification, and feature contracts.
- `tests/`: retained foundation tests and checks that pending routing is explicit.
- `docs/`: architecture, protocol, test scope, project state, and task writeups.
- `results/`: locally generated output; historical evidence is on `most-updated`.

This is a software testbed. Hardware deployment, bridge firmware, bench tooling,
PX4 integration, and completed dashboards are preserved on `most-updated`.
