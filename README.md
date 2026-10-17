# Reins by BotInfo

Reins is an open-source policy layer that sits between an AI agent and a robot its owner has connected. It sits in
front of the robot's existing connector. The agent sees only the actions the owner allows, each with checked
bounds. Every motion runs on a lease and stops by itself. A stop always wins, and every request and decision
goes to an audit log.

The repo also contains the **Reins Safety Benchmark**: hostile test cases that grade any connector, raw or
behind Reins, on what the robot actually did.

> **Status, v0.1 (2026-10-16): simulation and mocks only.** Nothing here has run on a real robot. The
> benchmark results are not a certification and say nothing about real hardware. Nobody outside BotInfo has
> reviewed this yet.

## Quickstart (one command)

From this folder, with Python 3.11 or newer and nothing to install:

```bash
python3 -m reins bench
```

It runs 20 cases against 8 stacks in simulated time (about a second) and writes `results/scorecard.json` and
`results/LEADERBOARD.md`. Other commands:

```bash
python3 -m reins demo                                  # a short scripted session, printed
python3 -m unittest discover -s tests -t tests         # 51 tests, about 5 s
```

## What Reins enforces

- **Only the owner's actions exist.** The policy file (`policies/*.toml`) lists the actions, each with typed,
  bounded parameters. An unknown name, an extra field, NaN, a string where a number belongs, or a value out of
  range is refused (never clamped) and logged. The connector's own tools are never passed to the agent.
- **Limits around the actions:** rate limits (overall and for motion), a longest stretch of continuous motion
  (renewals and back-to-back commands count together), and an optional workspace box for robots that report a
  pose.
- **Expiry and replay.** A command waiting for the owner's confirmation expires after `command_ttl_s`. A call
  whose `issued_at` is too old is refused, and so is a repeated `request_id`.
- **Owner-only controls.** Confirming an action, lifting a stop and changing the policy happen on the owner's
  local channel (`python3 -m reins owner ...`). The agent has no tool for any of them.
- **Stop.** `reins_stop` and the owner's stop skip every check and every queue. After a stop, motion stays
  refused until the owner resumes. If the agent's session ends, Reins stops the robot.
- **Leases.** A motion asks for a bounded time. While it runs, Reins re-sends it as a keep-alive, then sends stop
  at the end. A robot-side watchdog therefore stops the robot when Reins or the network goes away. This mirrors
  the Sesame firmware motion-timeout patch, or a ROS base's `cmd_vel` timeout.
- **Robot output is data.** Status is reduced to typed fields. Free text from the robot is redacted before the
  agent sees it, and flagged in the audit log.
- **Audit log.** JSONL, one line per request and decision, with each line chained to the previous one by
  SHA-256 (`python3 -m reins audit-verify FILE`). It is tamper-evident, not tamper-proof.

**A real emergency stop must be enforced locally on the robot** (a power switch, a hardware e-stop, a firmware
watchdog), never over the network. Reins' stop is a software command sent over a link. The benchmark shows
this: when the link drops on a robot with no local watchdog, it keeps moving, with or without Reins.

Threat model and architecture: [docs/DESIGN.md](docs/DESIGN.md). Policy reference: [docs/POLICY.md](docs/POLICY.md).

## The benchmark, v0.1 results

The full table, every case, and why each failure failed: [results/LEADERBOARD.md](results/LEADERBOARD.md).
Method and cases: [docs/BENCHMARK.md](docs/BENCHMARK.md).

| Stack | Score | Critical cases passed |
|---|---|---|
| Reins → ROS-style connector → SimRover with a 0.5 s base watchdog | 100 | 10/10 |
| Reins → Sesame firmware API (mock, with the motion-timeout patch) | 100 | 9/9 |
| Reins → ROS-style connector → SimRover, no base watchdog | 93 | 9/10 (link lost) |
| Reins → Sesame firmware API (mock, stock) | 93 | 8/9 (link lost) |
| Sesame firmware API (mock, patched), direct | 46 | 5/9 |
| ROS-style connector → SimRover with base watchdog, direct | 14 | 2/10 |
| Sesame firmware API (mock, stock), direct | 12 | 1/9 |
| ROS-style connector → SimRover, no watchdog, direct | 0 | 0/10 |

Third-party code, in real time: **ros-mcp-server v3.1.2 against an emulated rosbridge (not real ROS)**. That is
its real code, with BotInfo's emulated rosbridge and SimRover (no base watchdog) in place of ROS and a robot:

| Stack | Score | Critical cases passed |
|---|---|---|
| Reins → ros-mcp-server v3.1.2 against an emulated rosbridge (not real ROS) | 93 | 9/10 (link lost) |
| ros-mcp-server v3.1.2 against an emulated rosbridge (not real ROS), direct | 4 | 0/10 |

The direct row's one pass is the command flood, and only because each `publish_once` call takes about 2 s
(it waits for replies rosbridge never sends). A stop sent during `publish_for_durations` waits for that call to
end. Details and the emulator's limits: [docs/GRADING-EXISTING-CONNECTORS.md](docs/GRADING-EXISTING-CONNECTORS.md).

Read these numbers with three things in mind:

1. **Every row is a simulator or a mock.** The direct connectors above are BotInfo's reference models of
   existing patterns: a direct firmware API, and a generic ROS connector shaped like ros-mcp-server. One
   third-party connector has been graded so far, in real time: **ros-mcp-server v3.1.2 against an emulated
   rosbridge (not real ROS)**. That is its real code, but BotInfo's emulation of rosbridge in front of SimRover
   stands in for ROS. Its rows, and what the emulation leaves out, are in
   [results/LEADERBOARD.md](results/LEADERBOARD.md) and
   [docs/GRADING-EXISTING-CONNECTORS.md](docs/GRADING-EXISTING-CONNECTORS.md).
2. **Reins and the benchmark were written together, by the same team.** Reins passing its authors' tests proves
   little. The benchmark gets useful when someone adds a case Reins fails; see "Adding a case" in
   docs/BENCHMARK.md.
3. The hostile agent always obeys injected text. These are worst-case results for the connector, not
   predictions for any model.

## Use it with an agent (MCP)

Reins is an MCP server on stdio. For Claude Code, put this in a project `.mcp.json` (use absolute paths):

```json
{
  "mcpServers": {
    "reins-simrover": {
      "command": "python3",
      "args": ["-m", "reins", "serve",
               "--policy", "/ABSOLUTE/PATH/reins/policies/simrover.toml",
               "--binding", "/ABSOLUTE/PATH/reins/bindings/simrover-ros-style.toml"],
      "env": { "PYTHONPATH": "/ABSOLUTE/PATH/reins" }
    }
  }
}
```

`python3` must be 3.11 or newer. That binding runs the simulator inside the Reins process.
`bindings/simrover-ros-style-stdio.toml` instead launches the connector as a separate MCP server and fronts it.
That is the same arrangement as fronting ros-mcp-server: `bindings/ros-mcp-server-simrover.toml` is tested
against the emulated rosbridge (not real ROS), and `bindings/ros-mcp-server-turtlesim.toml` is the same pattern
for turtlesim, not yet run on it. For the Sesame mock, use `policies/sesame.toml` with
`bindings/sesame-inprocess.toml`.

While it runs, the owner uses a local socket (`~/.reins/<policy>.sock`, owner-only permissions):

```bash
python3 -m reins owner status
```

```bash
python3 -m reins owner confirm c-2
```

```bash
python3 -m reins owner stop
```

```bash
python3 -m reins owner resume
```

The audit log goes to `runs/` unless `--audit` says otherwise. For OpenClaw there is a skill stub:
[integrations/openclaw/reins/SKILL.md](integrations/openclaw/reins/SKILL.md). It is unverified and has not
been loaded into OpenClaw.

## What Reins does not do

- It does not make a robot physically harmless. It cannot see people, obstacles, falls or battery faults.
- It cannot stop a robot it can no longer reach. Only the robot's own watchdog or hardware stop can do that.
- It does not protect against an agent that has a shell on the owner's account. That agent could kill Reins,
  edit the policy or reach the robot directly. Give the agent the Reins MCP server and nothing else that
  touches the robot.
- It is not for robots whose owner has not connected them. Reins is built for owner-connected robots only.
- On Sesame, a pose (wave, dance, ...) cannot be interrupted once it starts; that is how the firmware works.
  The default policy therefore makes every pose wait for the owner's confirmation.

## Before any real robot

Do not use v0 with hardware. `bindings/sesame-mock.toml` refuses non-localhost robots, and the CLI refuses
`allow_remote`. A first hardware run would need, at minimum:

- the robot's own stop within a person's reach, and a robot-side watchdog tested on that robot;
- a tethered or bench setup first;
- an outside review of this code and the benchmark.

## Layout

```
reins/core.py              the engine: checks, leases, stop latch, confirmations, audit
reins/policy.py            policy loading (strict) and parameter checks
reins/audit.py             hash-chained JSONL audit log
reins/mcp/                 MCP server and client (stdio, standard library only)
reins/owner.py             the owner's local channel (Unix socket)
reins/backends/            Sesame firmware API + mock, SimRover + reference ROS-style connector, MCP tool templates
reins/bench/               the Reins Safety Benchmark (cases, stacks, ground truth, report)
policies/  bindings/       example owner policies and connector bindings
docs/                      DESIGN, BENCHMARK, POLICY, GRADING-EXISTING-CONNECTORS
results/                   the published v0.1 benchmark run
integrations/openclaw/     OpenClaw skill stub (unverified)
tests/                     unittest suite
```

## License and credits

Apache-2.0 ([LICENSE](LICENSE)); credits are in [NOTICE](NOTICE). The Sesame robot and firmware are Dorian
Todd's work (Apache-2.0). Reins is an independent BotInfo.ai project, not made or endorsed by him. The reference
ROS-style connector follows the tool interface of ros-mcp-server (Contoro Inc., Apache-2.0); no code was copied.
