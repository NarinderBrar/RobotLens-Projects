import argparse
import json
import sys
import time

from robotlens_mcp import launch_trainer, log, prepare_task, process_state, project_path, stop_all

FLAT_POLICY = "training/runs/microduck-balance-grid1-live-stage-02/policy.pt"
DEFAULT_STAGES = ["roller:25", "roller:20", "roller:16", "roller:12", "roller:08", "roller:05",
                  "dome:25", "dome:16", "dome:10", "dome:05",
                  "ball:25", "ball:16", "ball:10", "ball:06", "ball:04"]


def stage_files(kind, tag):
    if kind == "roller" and tag == "05":
        return "src/microduck/mujoco/real/microduck_seesaw_roller.xml", "training/microduck/seesaw_roller.task.json", "microduck-roller-r05"
    if kind == "ball" and tag == "04":
        return "src/microduck/mujoco/real/microduck_seesaw_balance.xml", "training/microduck/seesaw_balance.task.json", "microduck-seesaw-ball-r04"
    run_name = f"microduck-roller-r{tag}" if kind == "roller" else f"microduck-seesaw-{kind}-r{tag}"
    return f"src/microduck/mujoco/real/microduck_seesaw_{kind}_r{tag}.xml", f"training/microduck/seesaw_{kind}_r{tag}.task.json", run_name


def mode_args(kind, radius):
    if kind == "roller":
        return ["--seesaw-roller", "--roller-radius", str(radius)]
    return ["--seesaw-balance", "--ball-radius", str(radius)]


def update_count(checkpoint):
    try:
        return len(json.loads((project_path(checkpoint).parent / "training.json").read_text())["updates"])
    except (OSError, ValueError, KeyError):
        return 0


def train_stage(spec, previous, initial, args):
    kind, tag = spec.split(":")
    model, task, run_name = stage_files(kind, tag)
    checkpoint = f"training/runs/{run_name}/policy.pt"
    log("stage", spec, "from", previous or initial)
    if not prepare_task(model, task, args.grid, args.iterations):
        sys.exit(2)
    trainer_args = mode_args(kind, int(tag) / 100.0) + ["--steps", str(args.iterations * 500), "--rollout-steps", "500", "--checkpoint", checkpoint]
    if previous:
        trainer_args += ["--resume-from", previous]
        baseline = update_count(previous)
    else:
        trainer_args += ["--initialize-from", initial, "--critic-warmup", "5"]
        baseline = 0
    process = f"seesaw_{kind}_r{tag}_{int(time.time())}"
    if not launch_trainer(process, trainer_args, f"training/runs/{run_name}/ready"):
        sys.exit(3)
    metrics = project_path(f"training/runs/{run_name}/training.json")
    mastered = False
    while True:
        time.sleep(30)
        state = process_state(process)
        try:
            updates = json.loads(metrics.read_text())["updates"]
        except (OSError, ValueError, KeyError):
            updates = []
        new = [u for u in updates[baseline:] if u.get("recent_success_rate") is not None]
        if new:
            log(spec, "update", len(new), "length", round(new[-1]["recent_episode_length"], 1), "success", round(new[-1]["recent_success_rate"], 2))
        last = new[-args.streak:]
        if len(new) >= 10 and len(last) == args.streak and all(u["recent_success_rate"] >= args.success for u in last):
            mastered = True
            break
        if state["state"] != "running":
            log("trainer ended", state["state"], state["exit_code"], state["output_tail"][-300:])
            break
    stop_all()
    log("stage", spec, "mastered" if mastered else "not mastered")
    return checkpoint, mastered


def main():
    parser = argparse.ArgumentParser(description="Train the Microduck seesaw curriculum in a live RobotLens session.")
    parser.add_argument("stages", nargs="*", default=DEFAULT_STAGES, help="kind:radius_cm stages, kind is roller, dome, or ball")
    parser.add_argument("--initialize-from", default=FLAT_POLICY, help="checkpoint that starts the first stage")
    parser.add_argument("--resume", action="store_true", help="resume the first stage from --initialize-from instead of initializing")
    parser.add_argument("--grid", type=int, default=4)
    parser.add_argument("--iterations", type=int, default=300)
    parser.add_argument("--success", type=float, default=0.95)
    parser.add_argument("--streak", type=int, default=5)
    parser.add_argument("--retries", type=int, default=1, help="extra resumed attempts for a stage that is not mastered")
    args = parser.parse_args()
    previous = args.initialize_from if args.resume else None
    for spec in args.stages:
        for attempt in range(args.retries + 1):
            checkpoint, mastered = train_stage(spec, previous, args.initialize_from, args)
            previous = checkpoint
            if mastered:
                break
        if not mastered:
            sys.exit(4)
    log("all stages done")


if __name__ == "__main__":
    main()
