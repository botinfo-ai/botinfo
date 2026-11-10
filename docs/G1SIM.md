# Walk a simulated Unitree G1 through BotInfo

`botinfo/backends/g1sim.py` runs Unitree's published G1 walking policy in MuJoCo, behind a small connector whose
tools are named after Unitree's G1 LocoClient (`Move`, `StopMove`, `Damp`, `ZeroTorque`, plus `GetState`). With
`policies/g1sim.toml` and a binding, an AI agent can walk the simulated G1 through BotInfo's MCP server while the
owner keeps the stop on their own channel. Simulation only: nothing here has run on a real robot.

## Run it

```
pip install numpy==2.0.2 mujoco==3.3.7    # the versions the self-check's expected numbers come from
git clone https://github.com/unitreerobotics/unitree_rl_gym ~/unitree_rl_gym
git -C ~/unitree_rl_gym checkout 3e70454dae9562febade2949c9ebc57440365dcb
export BOTINFO_UNITREE_RL_GYM=~/unitree_rl_gym

python3 -m botinfo.backends.g1sim ~/unitree_rl_gym          # self-check: walks and stops the G1, prints a table
python3 -m botinfo check --policy policies/g1sim.toml --binding bindings/g1sim-loco.toml
python3 -m botinfo serve --policy policies/g1sim.toml --binding bindings/g1sim-loco.toml   # MCP on stdio
python3 -m botinfo owner stop          # from another terminal: the owner's stop
```

Set `viewer = true` in the binding to watch the robot in MuJoCo's viewer (it needs a display; on macOS start the
server with `mjpython -m botinfo serve ...`). Without a display the sim runs headless and says so on stderr.

The self-check's first three rows repeat the single run published in
[humanoid-stop-sim](https://github.com/botinfo-ai/humanoid-stop-sim) (`results/2026-11-08-stop_walk.md`) and must
match it to the millimetre. That run used numpy 2.0.2 and mujoco 3.3.7; with other versions those rows may not match
exactly and the self-check then prints `NOT as expected`. It prints the versions it ran on.

The sim (self-check, `botinfo check` and `botinfo serve` alike) refuses to start, with an error message, when
`motion.pt`'s sha256 is not the one this backend was tested with (`PINNED_POLICY_SHA256` in `g1sim.py`).

## What BotInfo's stop does here

| binding | BotInfo's stop (and every lease end) | the walking policy after it |
|---|---|---|
| `bindings/g1sim-loco.toml` | `StopMove`: velocity zero | keeps running and balancing |
| `bindings/g1sim-loco-damp.toml` | `Damp`: kp 0, kd 8 on every joint | off |

In humanoid-stop-sim, velocity zero kept the walking G1 upright at all 20 gait moments tried, and damping, zero
torque and holding the joints left it not upright at all 20 (pelvis 0.06-0.31 m above the floor 4 s later). There,
"upright" means the pelvis above 0.6 m and tilted under 30 degrees. That is why the default binding stops with
`StopMove`. The damp binding is there to see the other outcome with an agent in the loop: after a stop,
`botinfo_status` reports `upright` and `pelvis_z_m` (the connector's `GetState` calls the height `z`). This backend's
`upright` reads the pelvis height only (above 0.6 m); `GetState` also returns `tilt_deg`, which the bindings do not
pass on.

`Move` walks for 1 s, as Unitree's `LocoClient.Move()` does without `continous_move`, and BotInfo renews it every
0.2 s while a walk's lease runs. If BotInfo goes quiet, the simulated robot stops walking by itself within a second
and keeps balancing.

Stop receipts read `travel_mps`, the pelvis's net travel over the last second (`[settle]` in the bindings). Under
this policy the pelvis keeps moving a little at velocity zero: in humanoid-stop-sim's single run its horizontal speed
never stayed under 0.05 m/s for 0.48 s (12 samples at 25 Hz) in the 4 s after a zero-velocity stop. That is why the
receipt reads net travel rather than instantaneous speed.

## Limits

- One model: unitree_rl_gym's 12-joint G1 (legs; the arms are fixed shapes) and its `motion.pt`. Other G1
  controllers, the 23/29-joint models and Unitree's own locomotion service are not modelled.
- `Damp` and `ZeroTorque` are unitree_rl_gym's low-level commands (`create_damping_cmd`, `create_zero_cmd`), not
  the robot's FSM modes, whose behaviour is not in public code.
- BotInfo cannot stop a robot it can no longer reach: a real emergency stop must be on the robot.
