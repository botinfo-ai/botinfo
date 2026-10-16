# Grading existing connectors: what it takes

Status, 2026-10-16: **no third-party AI-to-robot connector has been graded yet.** The v0.1 leaderboard grades
BotInfo's reference connectors (models of existing patterns) and the Sesame firmware's own HTTP API, on a mock.
This page covers the two connectors we plan to grade first: what we learned by reading their code (nothing
was run), what an honest grading environment needs, and roughly how long it would take.

What this Mac has: Python 3.12 (Reins needs 3.11+), `uv`. **It has no Docker** (no docker, colima, podman or
orbstack) and no ROS.

## 1. ros-mcp-server (github.com/robotmcp/ros-mcp-server)

Read at commit `476591ac` (2026-09-23, v3.1.2). Apache-2.0, "Copyright [2025] [Contoro Inc.]".

How it works: Python 3.10+, FastMCP 2.x, `websocket-client`. It talks to ROS 1 or ROS 2 through rosbridge,
plain `ws://` (default 127.0.0.1:9090), with no TLS and no auth. Its transport is stdio by default (also HTTP).
It exposes 31 tools. Those that move or change the robot are all generic: the model picks the topic, the type
and the payload.

| Tool | Effect | Limits found in the code |
|---|---|---|
| `publish_once(topic, msg_type, msg)` | one message, any topic | none |
| `publish_for_durations(topic, msg_type, messages, durations, rate_hz)` | timed sequence, blocks the call | `rate_hz <= 100` only; no stop message at the end (`ros_mcp/tools/topics.py`, the `finally` only unadvertises) |
| `call_service(service_name, service_type, request, timeout)` | any service | none |
| `send_action_goal(...)`, `cancel_action_goal(...)` | any action | on timeout it stops waiting; we found no cancel in that path (read, not run) |
| `set_parameter(name, value)`, `delete_parameter(name)` | any parameter | none |
| `connect_to_robot(ip, port, ...)` | points the server at any host | marked not destructive |

Read-only tools (`subscribe_once`, `subscribe_for_duration`, `get_parameter`, service responses) return raw
robot data to the model, so they are a prompt-injection path (INJ-01 applies directly). There is no stop tool,
no confirmation and no topic allowlist. Safety guidance exists only as prompt text in the robot-spec YAML files.

How Reins would front it: `bindings/ros-mcp-server-turtlesim.toml` (draft, untested) maps `drive` onto
`publish_once` on `/turtle1/cmd_vel`, the stop onto a zero `Twist`, and status onto
`subscribe_once /turtle1/pose`. The agent never sees `publish_once`.

**Environment to grade it for real:**

1. A container runtime on the Mac: Colima (open source, no account), OrbStack (free for personal use only), or
   Docker Desktop (its subscription terms apply to companies). Installing one is the owner's call.
2. ROS 2 Humble + rosbridge + turtlesim, headless. Their own test image
   `tests/integration/docker/Dockerfile.ros2-humble` (`FROM ros:humble-ros-base`, `QT_QPA_PLATFORM=offscreen`,
   rosbridge + turtlesim + rosapi, port 9090) works on Apple Silicon. `ros:humble-ros-base` is published for
   arm64. Their demo image `osrf/ros:humble-desktop` is amd64 only and would run emulated.
3. `uvx ros-mcp --transport=stdio` (its own dependencies, installed in an isolated environment).
4. A real-time mode for the benchmark harness. Today every stack runs in simulated time in one process. For
   ros-mcp-server the stack is: the agent → (Reins) → ros-mcp-server over stdio → rosbridge → turtlesim.
   Ground truth comes from `/turtle1/pose` sampled at 50 Hz through a separate rosbridge client, plus a
   recorder for every topic, service and parameter the connector touches.
5. A turtlesim version of the reference limits. Its world runs 0 to 11.09 on both axes. Turtlesim also stops
   by itself about 1 s after the last `Twist`; that is our recollection of its source, to confirm once it runs.
   If so, it has a robot-side watchdog, and LNK-02 should pass with or without Reins.

**Effort:** about 1 day once a container runtime is installed: half a day for the real-time harness and
recorder, half a day to adapt the cases (topic names, limits) and check the draft binding.

**Cheaper first step, without Docker (about 1 day, lower fidelity):** a small rosbridge-protocol emulator in
Python (websocket + `advertise` / `publish` / `subscribe` / `call_service` / rosapi), backed by SimRover. That
would run the real ros-mcp-server code against our simulator. It grades the connector's behaviour, not ROS.
Label the results "ros-mcp-server v3.1.2 against an emulated rosbridge".

## 2. unitree-go2-mcp-server (github.com/lpigeon/unitree-go2-mcp-server)

Read at commit `e7a6737d` (2026-06-10). The LICENSE file is the Apache-2.0 text, but its copyright line is
still the template ("Copyright [yyyy] [name of copyright owner]").

How it works: Python 3.10+, `mcp` 1.7.1, stdio only. It needs a real Go2, Ubuntu with ROS 2 Humble or Foxy, and
`unitree_ros2`. Every movement shells out (`shell=True`) to
`ros2 topic pub /wirelesscontroller unitree_go/msg/WirelessController`. In other words, it imitates the
joystick. `server.py:7` hard-codes a `setup.sh` path under `/home/lpigeon/`.

| Tool | Effect | Limits found in the code |
|---|---|---|
| `pub_wirelesscontroller(linear_x, linear_y, yaw, pitch, keys, duration)` | stick and button input | sticks clamped to ±3.7, then scaled to ±1; `duration` not capped; `keys` takes any button combination |
| `stretch`, `shake_hands`, `love`, `pounce`, `jump_forward`, `sit_down`, `greet`, `dance` | each first sends "stand up from a fall", then its button code | none |
| `stand_up_from_a_fall`, `stop` | | `stop` = zero sticks, `keys = 0`, three times at 10 Hz (not a damping mode or e-stop) |
| `get_topics` | read-only | |

From reading the code (not run): a long `duration` blocks the server, so `stop()` probably cannot interrupt it.
The declared `websocket` dependency is not the `websocket-client` API the code uses, so the optional rosbridge
path may be broken.

**Environment to grade it without a Go2:**

- The simplest honest harness is a fake `ros2` executable on `PATH`. It parses
  `ros2 topic pub /wirelesscontroller ... '{lx: .., ly: .., rx: .., ry: .., keys: ..}'`, records each command
  with its time, and feeds a simple kinematic model (stick to body speed, named moves to timed poses). No ROS
  is needed. It grades what the connector sends (limits, durations, stop behaviour, blocking), not how a Go2
  walks.
- The hard-coded `setup.sh` path needs a local patched copy. Apache-2.0 allows that; we would mark the change.
  Do not send it back upstream without the owner's OK.
- Higher fidelity needs a Go2 locomotion policy that reads that topic (Isaac Lab or go2_omniverse; NVIDIA GPU,
  Linux). Not possible on this Mac. unitree_mujoco supports Go2, but only low-level commands; it publishes
  `wireless_controller` rather than consuming it.

**Effort:** about 1 to 1.5 days for the fake-`ros2` harness, the kinematic model and the case mapping (named moves
→ STP-03-style uninterruptible cases, `duration` → OOB-02, `keys` → UNK-01).

## 3. What counts toward "grades at least 2 existing connectors"

| Candidate | Graded today? | Fidelity |
|---|---|---|
| Sesame firmware HTTP API (Dorian Todd's firmware, stock and patched) | yes, on BotInfo's mock of the firmware | the firmware's API is real; the robot is a mock |
| ros-mcp-server v3.1.2 | no | real code; emulated rosbridge (1 day) or real rosbridge + turtlesim in Docker (1 day after a container runtime) |
| unitree-go2-mcp-server | no | real code; fake `ros2` command and a kinematic model (1 to 1.5 days) |

Whether the Sesame firmware API counts as an "existing robot connector" is a judgement call. It is the robot's
own control interface, not an AI connector.
