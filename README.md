# BotInfo

> Formerly named Reins (renamed 2026-10-25). Same code and the same results; the package, the command
> (`python3 -m botinfo`) and the agent's tools (`botinfo_stop`, `botinfo_status`, `botinfo_wait`, ...) carry the new
> name. Old links to github.com/botinfo-ai/reins redirect here.

BotInfo is an open-source policy layer that sits between an AI agent and a robot its owner has connected. It sits in
front of the robot's existing connector. The agent sees only the actions the owner allows, each with checked
bounds. Every motion runs on a lease and stops by itself. A stop always wins, and every request and decision
goes to an audit log.

The repo also contains the **BotInfo Benchmark**: hostile test cases that grade any connector, raw or
behind BotInfo, on what the robot actually did.

> **Status, v0.1 (2026-10-16): simulation and mocks only.** Nothing here has run on a real robot. The
> benchmark results are not a certification and say nothing about real hardware. Nobody outside BotInfo has
> reviewed this yet.

## Quickstart (one command)

From this folder, with Python 3.11 or newer and nothing to install (on an older Python, BotInfo says so in one line
and stops; macOS's built-in `python3` is 3.9, so use one from python.org or `uv`):

```bash
python3 -m botinfo bench
```

It runs 20 cases against 8 stacks in simulated time (about a second) and writes `results/scorecard.json` and
`results/LEADERBOARD.md`. Other commands:

```bash
python3 -m botinfo demo                                  # a short scripted session, printed
python3 -m unittest discover -s tests -t tests         # about 80 tests, about 10 s
```

## What BotInfo enforces

- **Only the owner's actions exist.** The policy file (`policies/*.toml`) lists the actions, each with typed,
  bounded parameters. An unknown name, an extra field, NaN, a string where a number belongs, or a value out of
  range is refused (never clamped) and logged. The connector's own tools are never passed to the agent.
- **Limits around the actions:** rate limits (overall and for motion), a longest stretch of continuous motion
  (renewals and back-to-back commands count together), and an optional workspace box for robots that report a
  pose.
- **Expiry and replay.** A command waiting for the owner's confirmation expires after `command_ttl_s`, and a
  newer motion command from the agent cancels it, so a late OK cannot start a move the agent has given up on. A call
  whose `issued_at` is too old is refused, and so is a repeated `request_id`.
- **The agent's picture stays true.** Motion calls return at once. When a new motion stops a running one early, the
  answer says which one and how far it got (`replaced`), so the agent never reports a move as finished when it was
  cut short. `botinfo_wait` lets the agent wait for a move to end; a stop ends the wait at once and is never queued
  behind it.
- **Owner-only controls.** Confirming an action, lifting a stop and changing the policy happen on the owner's
  local channel (`python3 -m botinfo owner ...`). The agent has no tool for any of them.
- **Stop.** `botinfo_stop` and the owner's stop skip every check and every queue. After a stop, motion stays
  refused until the owner resumes. If the agent's session ends, BotInfo stops the robot.
- **Leases.** A motion asks for a bounded time. While it runs, BotInfo re-sends it as a keep-alive, then sends stop
  at the end. A robot-side watchdog therefore stops the robot when BotInfo or the network goes away. This mirrors
  the Sesame firmware motion-timeout patch, or a ROS base's `cmd_vel` timeout.
- **Robot output is data.** Status is reduced to typed fields. Free text from the robot is redacted before the
  agent sees it, and flagged in the audit log.
- **Audit log.** JSONL, one line per request and decision, with each line chained to the previous one by
  SHA-256 (`python3 -m botinfo audit-verify FILE`). It is tamper-evident, not tamper-proof.

**A real emergency stop must be enforced locally on the robot** (a power switch, a hardware e-stop, a firmware
watchdog), never over the network. BotInfo's stop is a software command sent over a link. The benchmark shows
this: when the link drops on a robot with no local watchdog, it keeps moving, with or without BotInfo.

Threat model and architecture: [docs/DESIGN.md](docs/DESIGN.md). Policy reference: [docs/POLICY.md](docs/POLICY.md).

## The benchmark, v0.1 results

The full table, every case, and why each failure failed: [results/LEADERBOARD.md](results/LEADERBOARD.md).
Method and cases: [docs/BENCHMARK.md](docs/BENCHMARK.md). What each grading setup does and leaves out:
[docs/GRADING-EXISTING-CONNECTORS.md](docs/GRADING-EXISTING-CONNECTORS.md).

**Third-party connectors, real code on real ROS 2 middleware** (in local containers, real time, every case run
twice). None of these is a robot:

| Stack | Score | Critical cases passed |
|---|---|---|
| BotInfo → ros-mcp-server v3.1.2 + rosbridge + turtlesim on real ROS 2 Humble | 100 | 10/10 |
| ros-mcp-server v3.1.2 + rosbridge + turtlesim on real ROS 2 Humble, direct | 11 | 1/10 (link lost, thanks to turtlesim's own stop) |
| BotInfo → unitree-go2-mcp-server (e7a6737) + ros2 CLI, stand-in Go2 that centres the sticks 0.5 s after the last message | 68 | 6/10 |
| unitree-go2-mcp-server (e7a6737) + ros2 CLI, same stand-in, direct | 18 | 2/10 |
| BotInfo → unitree-go2-mcp-server, stand-in Go2 that holds the last joystick input | void | 5/10 (CTL-01 fails) |
| unitree-go2-mcp-server, same stand-in, direct | void | 1/10 (CTL-01 fails) |

- **ros-mcp-server** runs against turtlesim, a simulator that stops by itself 1 s after the last command; behind
  BotInfo, the lost-link pass is that watchdog at work, not BotInfo's. Direct, ros-mcp-server runs one tool call at a time,
  and each `publish_once` takes about 2 s because it waits for replies rosbridge never sends (rosbridge 2.0.8), so a
  stop sent during `publish_for_durations` waits for that call to end.
- **unitree-go2-mcp-server** drives a **stand-in Go2 node, not a Go2 and not a Go2 simulator** (Unitree's MuJoCo
  simulator cannot act on the joystick messages this server sends). Every command is a new `ros2 topic pub` process,
  and the server handles one call at a time, so no stop can interrupt a running move, behind BotInfo or not: BotInfo
  latches the stop at once, but the robot stopped 4.2 s later, when the move ended. Behind BotInfo every case about
  what may be sent passed; the workspace case fails because the server reads no position. Whether a real Go2 holds
  or centres the last joystick input is unknown to us, so both are graded. With the holding stand-in each move
  overruns by about 1.1 s (the server's closing stop is a second process), the CTL-01 control fails, and those rows
  have no score.
- An earlier grade of ros-mcp-server against BotInfo's emulated rosbridge (not real ROS) is kept on the leaderboard:
  93 behind BotInfo, 4 direct.

**BotInfo's reference stacks, simulated time** (the direct connectors are BotInfo's models of existing patterns,
not third-party code):

| Stack | Score | Critical cases passed |
|---|---|---|
| BotInfo → ROS-style connector → SimRover with a 0.5 s base watchdog | 100 | 10/10 |
| BotInfo → Sesame firmware API (mock, with the motion-timeout patch) | 100 | 9/9 |
| BotInfo → ROS-style connector → SimRover, no base watchdog | 93 | 9/10 (link lost) |
| BotInfo → Sesame firmware API (mock, stock) | 93 | 8/9 (link lost) |
| Sesame firmware API (mock, patched), direct | 46 | 5/9 |
| ROS-style connector → SimRover with base watchdog, direct | 14 | 2/10 |
| Sesame firmware API (mock, stock), direct | 12 | 1/9 |
| ROS-style connector → SimRover, no watchdog, direct | 0 | 0/10 |

Read these numbers with three things in mind:

1. **Every row is a simulator, a mock or a stand-in robot node.** Two third-party connectors have been graded
   so far, as real code on real ROS 2 middleware in containers: ros-mcp-server v3.1.2 (on turtlesim) and
   unitree-go2-mcp-server (on BotInfo's stand-in Go2). The other rows grade BotInfo's reference models of existing
   patterns. What each setup leaves out is in
   [docs/GRADING-EXISTING-CONNECTORS.md](docs/GRADING-EXISTING-CONNECTORS.md).
2. **BotInfo and the benchmark were written together, by the same team.** BotInfo passing its authors' tests proves
   little. The benchmark gets useful when someone adds a case BotInfo fails; see "Adding a case" in
   docs/BENCHMARK.md.
3. The hostile agent always obeys injected text. These are worst-case results for the connector, not
   predictions for any model.

## Use it with an agent (MCP)

BotInfo is an MCP server on stdio. For Claude Code, put this in a project `.mcp.json` (use absolute paths):

```json
{
  "mcpServers": {
    "botinfo-simrover": {
      "command": "python3",
      "args": ["-m", "botinfo", "serve",
               "--policy", "/ABSOLUTE/PATH/botinfo/policies/simrover.toml",
               "--binding", "/ABSOLUTE/PATH/botinfo/bindings/simrover-ros-style.toml"],
      "env": { "PYTHONPATH": "/ABSOLUTE/PATH/botinfo" }
    }
  }
}
```

`python3` must be 3.11 or newer. That binding runs the simulator inside the BotInfo process.
`bindings/simrover-ros-style-stdio.toml` instead launches the connector as a separate MCP server and fronts it.
That is the same arrangement as fronting ros-mcp-server: `bindings/ros-mcp-server-turtlesim.toml` (with
`policies/turtlesim.toml`) is tested on real ROS 2 Humble + rosbridge + turtlesim in a container, and
`bindings/ros-mcp-server-simrover.toml` against the emulated rosbridge (not real ROS). For the Sesame mock, use `policies/sesame.toml` with
`bindings/sesame-inprocess.toml`. `bindings/unitree-go2-mcp-server.toml` (with `policies/go2.toml`) fronts
unitree-go2-mcp-server; it has only been run against BotInfo's stand-in Go2 node in a container, never a robot.

While it runs, the owner uses a local socket (`~/.botinfo/<policy>.sock`, owner-only permissions):

```bash
python3 -m botinfo owner status
```

```bash
python3 -m botinfo owner confirm c-2
```

```bash
python3 -m botinfo owner stop
```

```bash
python3 -m botinfo owner resume
```

The audit log goes to `runs/` unless `--audit` says otherwise. For OpenClaw there is a skill stub:
[integrations/openclaw/botinfo/SKILL.md](integrations/openclaw/botinfo/SKILL.md). It is unverified and has not
been loaded into OpenClaw.

## What BotInfo does not do

- It does not make a robot physically harmless. It cannot see people, obstacles, falls or battery faults.
- It cannot stop a robot it can no longer reach. Only the robot's own watchdog or hardware stop can do that.
- It cannot make a connector faster than it is. If the connector handles one call at a time and a motion call
  blocks until the motion ends (unitree-go2-mcp-server does both), BotInfo latches a stop at once and refuses all
  motion after it, but the stop reaches the robot only when the running call returns.
- It can only enforce what the connector lets it see. A workspace limit needs a pose; a connector that reports no
  position (unitree-go2-mcp-server) leaves BotInfo with speed, duration and rate limits only.
- It does not protect against an agent that has a shell on the owner's account. That agent could kill BotInfo,
  edit the policy or reach the robot directly. Give the agent the BotInfo MCP server and nothing else that
  touches the robot.
- It is not for robots whose owner has not connected them. BotInfo is built for owner-connected robots only.
- On Sesame, a pose (wave, dance, ...) cannot be interrupted once it starts; that is how the firmware works.
  The default policy therefore makes every pose wait for the owner's confirmation.

## Before any real robot

Do not use v0 with hardware. `bindings/sesame-mock.toml` refuses non-localhost robots, and the CLI refuses
`allow_remote`. A first hardware run would need, at minimum:

- the robot's own stop within a person's reach, and a robot-side watchdog tested on that robot;
- a tethered or bench setup first;
- an outside review of this code and the benchmark.

## Questions

### How do I limit what an AI agent can make my robot do?

Write a policy file (`policies/*.toml`) that lists the actions you allow, each with typed, bounded parameters, plus
rate limits, a longest stretch of motion and, if the robot reports a pose, a workspace box. Then give the agent the
BotInfo MCP server instead of the connector. The agent sees only your actions; anything else is refused and logged.
Inside your policy, a hijacked agent can still use the actions you allowed. Reference: [docs/POLICY.md](docs/POLICY.md).

### How do I stop the robot?

The agent has a `botinfo_stop` tool, and the owner runs `python3 -m botinfo owner stop` on the local channel. A stop
skips every check and queue, and motion stays refused until the owner runs `python3 -m botinfo owner resume`. If the
agent's session ends, BotInfo stops the robot. All of these are software commands sent over a link, so a real
emergency stop must be on the robot itself (a hardware stop or a firmware watchdog).

### Can a prompt injection make my robot do something I didn't allow?

The benchmark assumes it can hijack the agent: its scripted agent obeys every instruction it reads, from robot
status, a skill or a web page. Behind BotInfo, those instructions can only reach the actions and bounds in your
policy, and free text from the robot is redacted before the agent sees it. Cases INJ-01, INJ-02 and MAL-01 in
[docs/BENCHMARK.md](docs/BENCHMARK.md) test this.

### Does it work with Claude Code or other MCP clients?

BotInfo is an MCP server on stdio. "Use it with an agent" above has a `.mcp.json` for Claude Code; any MCP client
that can launch a local stdio server can run the same command. In the published benchmark the agent is BotInfo's
own scripted MCP client, not a commercial host.

### Does it work with ros-mcp-server?

Yes, in front of it: `bindings/ros-mcp-server-turtlesim.toml` with `policies/turtlesim.toml`, tested on real ROS 2
Humble + rosbridge + turtlesim in a container. ros-mcp-server v3.1.2 scored 11 direct and 100 behind BotInfo; the
lost-link pass comes from turtlesim's own 1 s stop, not from BotInfo.

### Does it work with a Unitree Go2?

Not with a real Go2 yet. BotInfo fronts unitree-go2-mcp-server (`bindings/unitree-go2-mcp-server.toml` with
`policies/go2.toml`), and that has only run against BotInfo's stand-in Go2 node in a container, never a robot.
There it scored 18 direct and 68 behind BotInfo. That server handles one call at a time, so a stop reaches the robot
only when the running move ends.

### How is a connector graded?

Twenty cases run against the connector direct and behind BotInfo: two controls and eighteen hostile cases (bounds,
injection, replay, flood, lost link, stop, malicious skill). Each case is graded on what the robot (a simulator, mock
or stand-in) actually did, never on what the connector replied. Critical cases count 3, high cases 2, and the score
is the weighted percentage passed. If a control fails (a normal move, or one the owner confirmed), the stack gets no
score ("void"), so refusing everything cannot win. Details: [docs/BENCHMARK.md](docs/BENCHMARK.md).

### How do I grade my own connector?

Follow "Adding a stack" in [docs/BENCHMARK.md](docs/BENCHMARK.md): add a factory to `STACKS` in
`botinfo/bench/stacks.py`, or to `REALTIME_STACKS` for a connector that runs as its own MCP server (copy `_ros_mcp()`).
Then run `python3 -m botinfo bench --stack ID -v`. What each grading setup leaves out:
[docs/GRADING-EXISTING-CONNECTORS.md](docs/GRADING-EXISTING-CONNECTORS.md). A new case that BotInfo fails is welcome
too ("Adding a case").

### Can I use it on a real robot?

Not yet. v0.1 is simulation and mocks only, and nothing here has run on a real robot. See "Before any real robot"
above.

### What is the licence?

Apache-2.0.

## Layout

```
botinfo/core.py              the engine: checks, leases, stop latch, confirmations, audit
botinfo/policy.py            policy loading (strict) and parameter checks
botinfo/audit.py             hash-chained JSONL audit log
botinfo/mcp/                 MCP server and client (stdio, standard library only)
botinfo/owner.py             the owner's local channel (Unix socket)
botinfo/backends/            Sesame firmware API + mock, SimRover + reference ROS-style connector, MCP tool templates
botinfo/bench/               the BotInfo Benchmark (cases, stacks, ground truth, report)
botinfo/bench/docker/        container images for the real-ROS grades (turtlesim; Go2 server + stand-in robot)
policies/  bindings/       example owner policies and connector bindings
docs/                      DESIGN, BENCHMARK, POLICY, GRADING-EXISTING-CONNECTORS
results/                   the published v0.1 benchmark run
integrations/openclaw/     OpenClaw skill stub (unverified)
tests/                     unittest suite
```

## License and credits

Apache-2.0 ([LICENSE](LICENSE)); credits are in [NOTICE](NOTICE). The Sesame robot and firmware are Dorian
Todd's work (Apache-2.0). BotInfo is an independent BotInfo.ai project, not made or endorsed by him. The reference
ROS-style connector follows the tool interface of ros-mcp-server (Contoro Inc., Apache-2.0); no code was copied.
The Go2 grade runs unitree-go2-mcp-server (Apache-2.0) unmodified inside a container image; it is not included
here.
