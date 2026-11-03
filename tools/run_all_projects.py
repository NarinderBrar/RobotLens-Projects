import argparse
import datetime
import json
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

from quickstart_processes import OwnedProcesses, clean_session_records, reap_children


ROOT = Path(__file__).resolve().parents[1]


def discover_projects(root, names):
    projects = {path.parent.parent.name: path for path in sorted(root.glob("*/scripts/quickstart.py"))}
    unknown = set(names) - projects.keys()
    if unknown:
        raise ValueError("Unknown projects: " + ", ".join(sorted(unknown)))
    return [path for name, path in projects.items() if not names or name in names]


def write_summary(directory, projects, results, status):
    counts = {label: sum(r["result"] == label for r in results) for label in ("PASS", "FAIL", "INTERRUPTED")}
    data = {"projects": len(projects), "completed": len(results), "status": status,
            "counts": counts, "results": results}
    (directory / "summary.json").write_text(json.dumps(data, indent=2) + "\n")
    lines = ["# Project quickstarts", "", f"Completed {len(results)}/{len(projects)}: "
             f"{counts['PASS']} PASS, {counts['FAIL']} FAIL, {counts['INTERRUPTED']} INTERRUPTED.",
             f"Status: {status}.", "", "| Project | Result | Seconds | Cleanup | Evidence |",
             "|---|---|---:|---|---|"]
    for result in results:
        name = result["project"]
        lines.append(f"| {name} | {result['result']} | {result['seconds']} | "
                     f"{'PASS' if result['cleanup_ok'] else 'FAIL'} | [console]({name}/console.log) |")
    (directory / "summary.md").write_text("\n".join(lines) + "\n")


def run_project(script, directory, python, timeout, live_sessions, session_directory):
    name = script.parent.parent.name
    output = directory / name
    output.mkdir()
    report = script.parent.parent / "evidence/quickstart/report.json"
    previous_time = report.stat().st_mtime_ns if report.exists() else None
    if report.exists():
        shutil.copyfile(report, output / "prior-report.json")
    started = time.monotonic()
    owned = OwnedProcesses()
    timed_out = interrupted = False
    with (output / "console.log").open("w") as log:
        process = subprocess.Popen([python, "-u", str(script)], cwd=script.parent.parent,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while process.poll() is None:
                owned.refresh()
                if time.monotonic() - started >= timeout:
                    timed_out = True
                    break
                time.sleep(0.5)
        except KeyboardInterrupt:
            interrupted = True
        finally:
            cleanup = owned.cleanup()
            if process.poll() is None:
                process.kill()
            process.wait()
            reap_children()
    records = clean_session_records(owned.identities, session_directory())
    active = [session.pid for session in live_sessions()]
    cleanup_ok = not any(cleanup.values()) and not records["retained"] and not active
    payload, report_error = None, None
    if report.exists() and report.stat().st_mtime_ns != previous_time:
        shutil.copyfile(report, output / "report.json")
        try:
            payload = json.loads(report.read_text())
        except (ValueError, OSError) as error:
            report_error = str(error)
    passed = process.returncode == 0 and not timed_out and cleanup_ok and isinstance(payload, dict) and payload.get("passed") is True
    result = {"project": name, "result": "INTERRUPTED" if interrupted else "PASS" if passed else "FAIL",
              "exit_code": process.returncode, "seconds": round(time.monotonic() - started, 1),
              "timed_out": timed_out, "cleanup_ok": cleanup_ok, "cleanup": cleanup,
              "session_records": records, "remaining_sessions": active, "report_error": report_error}
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run every project's existing quickstart sequentially")
    parser.add_argument("--list", action="store_true", help="list projects without launching anything")
    parser.add_argument("--projects", nargs="+", default=[], metavar="NAME", help="run selected project names")
    parser.add_argument("--timeout", type=float, default=1800, metavar="SECONDS", help="per-project limit (default: 1800)")
    parser.add_argument("--python", default=sys.executable, help="interpreter for project quickstarts")
    parser.add_argument("--report-dir", type=Path, help="new directory for this run's evidence")
    parser.add_argument("--stop-on-failure", action="store_true")
    args = parser.parse_args(argv)
    if not 0 < args.timeout < float("inf"):
        parser.error("--timeout must be a finite positive number")
    try:
        projects = discover_projects(ROOT, args.projects)
    except ValueError as error:
        parser.error(str(error))
    if not projects:
        parser.error("No project quickstarts found")
    if args.list:
        print("\n".join(path.parent.parent.name for path in projects))
        print(f"{len(projects)} project(s)")
        return 0
    sys.path.insert(0, str(ROOT.parent / "RobotLens-Automation/scripts"))
    try:
        from mcp_client import live_sessions, session_directory
        from orchestrate import become_child_subreaper
    except ImportError as error:
        print(f"Automation harness unavailable: {error}", file=sys.stderr)
        return 2
    if live_sessions():
        print("Close the existing RobotLens session before starting this batch.", file=sys.stderr)
        return 2
    if not become_child_subreaper():
        print("Cannot establish owned-process cleanup.", file=sys.stderr)
        return 2
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    directory = args.report_dir or ROOT / "reports" / f"quickstarts-{stamp}"
    results, status, code = [], "running", 0
    created = False
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    previous_handler = signal.signal(signal.SIGTERM, interrupt)
    try:
        directory.mkdir(parents=True, exist_ok=False)
        created = True
        print(f"Reports: {directory}", flush=True)
        write_summary(directory, projects, results, status)
        for index, script in enumerate(projects, 1):
            if live_sessions():
                raise RuntimeError("Unexpected live RobotLens session; stopping batch")
            print(f"START {index}/{len(projects)} {script.parent.parent.name}", flush=True)
            result = run_project(script, directory, args.python, args.timeout, live_sessions, session_directory)
            results.append(result)
            print(f"{result['result']} {result['project']} ({result['seconds']}s)", flush=True)
            write_summary(directory, projects, results, status)
            if result["result"] == "INTERRUPTED":
                status, code = "interrupted", 130
                break
            if not result["cleanup_ok"]:
                status, code = "cleanup_failed", 2
                break
            if result["result"] != "PASS":
                code = 1
                if args.stop_on_failure:
                    status = "stopped_on_failure"
                    break
        else:
            status = "complete"
    except KeyboardInterrupt:
        status, code = "interrupted", 130
    except (OSError, RuntimeError) as error:
        print(str(error), file=sys.stderr)
        status, code = "error", 2
    finally:
        owned = OwnedProcesses()
        cleanup = owned.cleanup()
        reap_children()
        records = clean_session_records(owned.identities, session_directory())
        active = [session.pid for session in live_sessions()]
        if any(cleanup.values()) or records["retained"] or active:
            status, code = "cleanup_failed", 2
        if created:
            write_summary(directory, projects, results, status)
            (directory / "final-cleanup.json").write_text(json.dumps(
                {"cleanup": cleanup, "session_records": records, "live_sessions": active}, indent=2) + "\n")
        signal.signal(signal.SIGTERM, previous_handler)
    return code


if __name__ == "__main__":
    sys.exit(main())
