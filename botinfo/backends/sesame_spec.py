"""What the Sesame firmware actually accepts, read from its source.

Copied from BotInfo's sesame-kit v0.2 (src/firmware_spec.py, same copyright holder, Apache-2.0) on 2026-10-16,
unchanged except this paragraph. sesame-kit's tests re-derive these facts from the firmware source; this copy
is not re-checked here.

Source: github.com/dorianborian/sesame-robot (Apache-2.0, by Dorian Todd), commit d106cbc8 (2026-09-12),
files firmware/sesame-firmware-main.ino, firmware/movement-sequences.h, firmware/face-bitmaps.h.
Line numbers are cited in API-NOTES.md. test_kit.py re-derives every list and timing below from those files
(when vendor/sesame-robot is present) so a firmware update that changes them fails the tests.

Nothing here talks to a robot. It only describes one.
"""
from __future__ import annotations

FIRMWARE_REPO = "https://github.com/dorianborian/sesame-robot"
FIRMWARE_COMMIT = "d106cbc8e158c982f408db421134f23273c5f51f"

# ------------------------------------------------------------------ commands
# loop() dispatch, sesame-firmware-main.ino:762-783. The JSON API (handleApiCommand, :303-387) stores ANY
# string in currentCommand and answers 200 "Command executed"; loop() only acts on these names. An unknown
# name is silently ignored but stays in currentCommand until the next command: the firmware does NOT
# validate. This kit validates instead.
MOVE_COMMANDS = ("forward", "backward", "left", "right")       # repeat until "stop" (see below)
POSTURE_COMMANDS = ("stand", "rest")                            # one-shot, loop() clears them
POSE_COMMANDS = ("wave", "dance", "swim", "point", "pushup", "bow", "cute", "freaky",
                 "worm", "shake", "shrug", "dead", "crab")      # one-shot, each pose clears itself
STOP_COMMAND = "stop"                                           # handleApiCommand :377-380
FIRMWARE_COMMANDS = MOVE_COMMANDS + POSTURE_COMMANDS + POSE_COMMANDS + (STOP_COMMAND,)

# ------------------------------------------------------------------ faces
# FACE_LIST, face-bitmaps.h:13-50, plus "default" added in faceEntries (sesame-firmware-main.ino:140-145).
FIRMWARE_FACES = ("walk", "rest", "swim", "dance", "wave", "point", "stand", "cute", "pushup", "freaky",
                  "bow", "worm", "shake", "shrug", "dead", "crab", "defualt", "idle", "idle_blink",
                  "happy", "talk_happy", "sad", "talk_sad", "angry", "talk_angry", "surprised",
                  "talk_surprised", "sleepy", "talk_sleepy", "love", "talk_love", "excited", "talk_excited",
                  "confused", "talk_confused", "thinking", "talk_thinking", "default")
# Faces registered in FACE_LIST whose base bitmap is NOT defined in face-bitmaps.h at this commit: setFace()
# finds zero frames and falls back to "default", which has no bitmap either (:903-934), so the screen does
# not change. ("defualt" is the firmware's own spelling.)
FACES_WITHOUT_BITMAP = ("stand", "defualt", "default")

# ------------------------------------------------------------------ settings and timing
# Defaults, sesame-firmware-main.ino:117-119 (changeable at runtime with /setSettings; this kit never does).
DEFAULT_SETTINGS = {"frameDelay": 100, "walkCycles": 10, "motorCurrentDelay": 20}
STAND_WRITES = 8          # runStandPose(): 8 setServoAngle calls (movement-sequences.h:77-89)

# One-shot poses: (servo writes, fixed delay ms), counted from movement-sequences.h:71-323, INCLUDING the
# runStandPose() calls inside them. Each servo write costs motorCurrentDelay ms (setServoAngle, :1051-1057).
POSE_COST = {
    "stand": (8, 0), "rest": (8, 0),
    "wave": (29, 3100), "dance": (56, 3300), "swim": (48, 3200), "point": (16, 2000),
    "pushup": (36, 5100), "bow": (26, 3800), "cute": (44, 3400), "freaky": (28, 2800),
    "worm": (64, 3400), "shake": (42, 3400), "shrug": (24, 2700), "dead": (12, 200), "crab": (64, 3200),
}
# Poses that end standing (they finish with runStandPose(1), which also starts the idle face). "dead" ends
# lying with its legs folded; "rest" ends with every servo at 90 degrees.
POSES_ENDING_STANDING = ("stand", "wave", "dance", "swim", "point", "pushup", "bow", "cute", "freaky",
                         "worm", "shake", "shrug", "crab")

# Walking/turning: (initial writes, initial frames, writes per gait cycle, frames per gait cycle).
# A "frame" is pressingCheck(cmd, frameDelay): the robot holds for frameDelay ms and checks whether
# currentCommand changed; if it did, it stands up and returns (:1059-1072). runWalkPose and friends run
# walkCycles cycles, stand, and because they do not clear currentCommand, loop() starts them again:
# a move continues until a stop (or another command) arrives. There is no timeout in the firmware.
MOVE_COST = {
    "forward": (4, 1, 16, 6),     # runWalkPose        movement-sequences.h:326-351
    "backward": (0, 1, 16, 6),    # runWalkBackward    :354-376
    "left": (0, 0, 16, 8),        # runTurnLeft        :379-403
    "right": (0, 0, 16, 8),       # runTurnRight       :405-429
}

# Faces each pose shows while it runs (setFaceWithMode calls in movement-sequences.h). runStandPose(1) at the
# end shows "stand" (no bitmap) and then enterIdle() switches to "idle".
POSE_FACE = {"stand": "idle", "rest": "rest", "wave": "wave", "dance": "dance", "swim": "swim", "point": "point",
             "pushup": "pushup", "bow": "bow", "cute": "cute", "freaky": "freaky", "worm": "worm",
             "shake": "shake", "shrug": "shrug", "dead": "dead", "crab": "crab",
             "forward": "walk", "backward": "walk", "left": "walk", "right": "walk"}


def pose_seconds(name: str, settings: dict | None = None) -> float:
    """How long a one-shot pose takes on the robot, from the firmware's own delays."""
    s = {**DEFAULT_SETTINGS, **(settings or {})}
    writes, delay_ms = POSE_COST[name]
    return (writes * s["motorCurrentDelay"] + delay_ms) / 1000.0


def gait_cycle_seconds(move: str, settings: dict | None = None) -> float:
    """One walking or turning cycle (one 'step' in this kit)."""
    s = {**DEFAULT_SETTINGS, **(settings or {})}
    _, _, w, f = MOVE_COST[move]
    return (w * s["motorCurrentDelay"] + f * s["frameDelay"]) / 1000.0


def move_start_seconds(move: str, settings: dict | None = None) -> float:
    s = {**DEFAULT_SETTINGS, **(settings or {})}
    w, f, _, _ = MOVE_COST[move]
    return (w * s["motorCurrentDelay"] + f * s["frameDelay"]) / 1000.0


def move_seconds(move: str, steps: int, settings: dict | None = None) -> float:
    """Time to walk/turn `steps` gait cycles before the stop is sent."""
    return move_start_seconds(move, settings) + steps * gait_cycle_seconds(move, settings)


def stand_seconds(settings: dict | None = None) -> float:
    s = {**DEFAULT_SETTINGS, **(settings or {})}
    return STAND_WRITES * s["motorCurrentDelay"] / 1000.0


# ------------------------------------------------------------------ BotInfo safety patch (firmware-patch/)
# NOT part of Dorian's firmware at d106cbc8: our proposed change (firmware-patch/sesame-safety-timeout.patch,
# compiled but never run on a robot). With it, a forward/backward/left/right sent to /api/command stops (and the
# robot stands) if no /api/command arrives for motionTimeout ms; any accepted /api/command resets the timer, so
# re-sending the move is the keep-alive. Unknown command names get 400 and change nothing. A patched robot reports
# "motionTimeout" in /getSettings; stock firmware does not. Values cross-checked against the patch by test_kit.py.
PATCH_MOTION_TIMEOUT_MS = 3000                   # MOTION_TIMEOUT_MS, firmware-patch/firmware/motion-watchdog.h
PATCH_MOTION_TIMEOUT_RANGE = (500, 60000)        # /setSettings?motionTimeout= is clamped to this; 0 = off (compile time)
PATCH_UNKNOWN_COMMAND_ERROR = "Unknown command"  # 400 {"error":"Unknown command","command":"<name>"}


# Plausible ranges for the robot's runtime settings. If /getSettings reports something outside them, this kit
# refuses to time walks from them (someone changed the robot; the step timing would be wrong).
SANE_SETTINGS = {"frameDelay": (20, 1000), "walkCycles": (1, 100), "motorCurrentDelay": (0, 200)}


def settings_problem(settings: dict) -> str | None:
    for k, (lo, hi) in SANE_SETTINGS.items():
        v = settings.get(k)
        if isinstance(v, bool) or not isinstance(v, int) or not lo <= v <= hi:
            return f"robot setting {k}={v!r} is outside {lo}..{hi}"
    return None
