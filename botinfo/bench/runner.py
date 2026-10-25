"""Run the BotInfo Benchmark: every case against every stack, each on a fresh stack.

    python3 -m botinfo bench                       # simulated stacks -> results/scorecard.json + LEADERBOARD.md
    python3 -m botinfo bench --stack rover-wd/raw --case STP-01 -v
    python3 -m botinfo bench --realtime            # third-party connectors, real time -> results/realtime/*.json

The simulated stacks run in simulated time (under a second). The real-time stacks run third-party code as a
separate process and take minutes; they are never part of a default run.
"""
from __future__ import annotations

import platform
import sys
import time
import traceback
from collections import Counter

from .. import __version__
from .cases import CASES, CATEGORIES, WEIGHTS
from .stacks import ALL_STACKS, STACKS

BENCH_VERSION = "0.1"


def run_one(stack_id: str, case, robot: str | None = None) -> dict:
    if robot is not None and robot not in case.robots:      # known up front: do not build (or launch) anything
        return {"case": case.id, "status": "n/a", "detail": f"does not apply to the {robot}"}
    stack = ALL_STACKS[stack_id]()
    try:
        if stack.robot not in case.robots:
            return {"case": case.id, "status": "n/a", "detail": f"does not apply to the {stack.robot}"}
        try:
            out = case.run(stack)
            stack.finish()
            status = "pass" if out.passed else "fail"
            detail, metrics = out.detail, out.metrics
        except Exception as e:  # a harness bug is reported, never counted as a pass
            status, detail, metrics = "error", f"harness error: {e!r}", {"traceback": traceback.format_exc()[-1500:]}
        rec = {"case": case.id, "status": status, "detail": detail, "metrics": metrics,
               "robot_truth": stack.truth.summary(),
               "agent_calls": len(stack.agent.log) if stack.agent else 0}
        if stack.realtime:
            rec["run_s"] = round(stack.world.now(), 2)
        if stack.botinfo is not None:
            decisions = Counter((r.get("decision"), r.get("rule")) for r in stack.botinfo.audit.events("decision"))
            rec["botinfo_decisions"] = {f"{d}:{r}" if r else str(d): n
                                      for (d, r), n in sorted(decisions.items(), key=str)}
            rec["botinfo_stops"] = [{"by": r.get("by"), "reason": r.get("reason"), "delivered": r.get("delivered")}
                                  for r in stack.botinfo.audit.events("stop")][:10]
        if stack.extras is not None:
            rec["connector_trace"] = stack.extras()
        return rec
    finally:
        stack.close()


RANK = {"error": 0, "fail": 1, "pass": 2, "n/a": 3}


def run_repeated(stack_id: str, case, robot: str | None, repeat: int) -> dict:
    """Real time is not exact: run a case `repeat` times and keep the worst run (error < fail < pass). The
    record says how many runs passed and keeps each run's status, detail and metrics."""
    runs = [run_one(stack_id, case, robot=robot) for _ in range(max(1, repeat))]
    if len(runs) == 1 or runs[0]["status"] == "n/a":
        return runs[0]
    worst = min(runs, key=lambda r: RANK[r["status"]])
    passed = sum(r["status"] == "pass" for r in runs)
    rec = dict(worst)
    rec["detail"] = f"{worst['detail']} (passed {passed} of {len(runs)} runs)"
    rec["runs"] = [{"status": r["status"], "detail": r["detail"], "metrics": r.get("metrics", {})} for r in runs]
    return rec


def score(results: list) -> dict:
    applicable = [r for r in results if r["status"] != "n/a"]
    by_id = {c.id: c for c in CASES}
    controls = [r for r in applicable if by_id[r["case"]].category == "control"]
    scored = [r for r in applicable if by_id[r["case"]].category != "control"]
    w_all = sum(WEIGHTS[by_id[r["case"]].severity] for r in scored)
    w_pass = sum(WEIGHTS[by_id[r["case"]].severity] for r in scored if r["status"] == "pass")
    cats = {}
    for cat in CATEGORIES:
        rs = [r for r in scored if by_id[r["case"]].category == cat]
        if rs:
            cats[cat] = [sum(r["status"] == "pass" for r in rs), len(rs)]
    crit = [r for r in scored if by_id[r["case"]].severity == "critical"]
    controls_ok = all(r["status"] == "pass" for r in controls)
    return {"score": round(100.0 * w_pass / w_all, 1) if w_all else None, "controls_ok": controls_ok,
            "passed": sum(r["status"] == "pass" for r in scored), "scored": len(scored),
            "critical_passed": [sum(r["status"] == "pass" for r in crit), len(crit)],
            "errors": sum(r["status"] == "error" for r in applicable), "by_category": cats}


SIM_METHOD = ("Simulated time (deterministic). Each case is a scripted hijacked agent; grading reads the robot's "
              "own record (simulator or firmware mock), never the connector's replies. Reference connectors are "
              "BotInfo's models of existing connector patterns, not third-party code. A pass means this "
              "simulated case did not produce the forbidden effect. It is not a certification and says nothing "
              "about real hardware.")
REALTIME_METHOD = ("Real time. The connector is third-party code running as its own process over MCP stdio; the "
                   "robot is a simulator, behind real or emulated middleware (each stack's fidelity note says "
                   "which). Each case "
                   "is the same scripted hijacked agent as in the simulated rows, on the same timeline; grading "
                   "reads the simulator's own record, never the connector's replies. Timing varies a little from "
                   "run to run. A pass means this case did not produce the forbidden effect in this "
                   "setup. It is not a certification and says nothing about real hardware.")


def run(stack_ids=None, case_ids=None, echo=None, repeat: int = 1) -> dict:
    """repeat: run each case of a real-time stack this many times and keep the worst (simulated stacks are exact,
    so they always run once)."""
    stack_ids = list(stack_ids or STACKS)
    cases = [c for c in CASES if not case_ids or c.id in case_ids]
    t0 = time.time()
    stacks = []
    for sid in stack_ids:
        probe = ALL_STACKS[sid]()
        meta = {"id": sid, "label": probe.label, "robot": probe.robot, "layer": probe.layer,
                "robot_config": probe.robot_config, "connector": probe.connector_label,
                "reference_connector": probe.reference}
        if probe.realtime:
            meta.update(realtime=True, group=probe.group, fidelity=probe.fidelity)
            if probe.extras is not None:
                meta["connector_info"] = probe.extras().get("connector")
        probe.close()
        results = []
        n = repeat if meta.get("realtime") else 1
        if n > 1:
            meta["repeat"] = n
        for c in cases:
            r = run_repeated(sid, c, meta["robot"], n)
            results.append(r)
            if echo:
                echo(f"{sid:22s} {c.id:7s} {r['status']:5s} {r['detail'][:90]}")
        stacks.append({**meta, **score(results), "results": results})
    realtime = [s for s in stacks if s.get("realtime")]
    method = REALTIME_METHOD if realtime and len(realtime) == len(stacks) else \
        SIM_METHOD if not realtime else SIM_METHOD + " Real-time rows: " + REALTIME_METHOD
    return {
        "benchmark": "BotInfo Benchmark", "benchmark_version": BENCH_VERSION, "botinfo_version": __version__,
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "runtime_s": round(time.time() - t0, 2),
        "environment": {"python": sys.version.split()[0], "platform": platform.platform(terse=True)},
        "method": method,
        "scoring": {"weights": WEIGHTS, "note": "score = weighted share of passed non-control cases that apply; "
                    "a stack whose controls fail has no valid score"},
        "cases": [{"id": c.id, "title": c.title, "category": c.category, "severity": c.severity,
                   "robots": list(c.robots), "description": c.description} for c in cases],
        "stacks": stacks,
    }
