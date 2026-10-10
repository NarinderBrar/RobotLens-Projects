import argparse
import time

from robotlens_mcp import launch_trainer, log, prepare_task, process_state, stop_all

FINAL_POLICY = "training/runs/microduck-seesaw-ball-r04/policy.pt"


def main():
    parser = argparse.ArgumentParser(description="Run a trained Microduck seesaw policy in a live RobotLens session.")
    parser.add_argument("--checkpoint", default=FINAL_POLICY)
    parser.add_argument("--model", default="src/microduck/mujoco/real/microduck_seesaw_balance.xml")
    parser.add_argument("--task", default="training/microduck/seesaw_balance.task.json")
    parser.add_argument("--mode", choices=["ball", "roller"], default="ball")
    parser.add_argument("--radius", type=float, default=0.04, help="support radius in metres, matching --model")
    parser.add_argument("--grid", type=int, default=8)
    parser.add_argument("--iterations", type=int, default=100000, help="500-transition iterations; the default runs for days")
    parser.add_argument("--device", choices=["gpu", "cpu"], default="gpu", help="where RobotLens runs the physics")
    parser.add_argument("--seed", type=int, default=31415)
    parser.add_argument("--stop", action="store_true", help="stop the running policy and exit")
    args = parser.parse_args()
    if args.stop:
        stop_all()
        return
    if not prepare_task(args.model, args.task, args.grid, args.iterations, args.device):
        raise SystemExit(2)
    run_dir = args.checkpoint.rsplit("/", 1)[0]
    mode = ["--seesaw-roller", "--roller-radius"] if args.mode == "roller" else ["--seesaw-balance", "--ball-radius"]
    trainer_args = mode + [str(args.radius), "--steps", str(args.iterations * 500), "--rollout-steps", "500", "--seed", str(args.seed),
                           "--checkpoint", args.checkpoint, "--evaluation-report", f"{run_dir}/run_policy.json", "--evaluate"]
    process = f"seesaw_policy_{int(time.time())}"
    if not launch_trainer(process, trainer_args, f"{run_dir}/eval-ready"):
        raise SystemExit(3)
    try:
        while process_state(process)["state"] == "running":
            time.sleep(10)
        state = process_state(process)
        log("finished", state["exit_code"], state["output_tail"][-600:])
    except KeyboardInterrupt:
        log("stopping")
    stop_all()


if __name__ == "__main__":
    main()
