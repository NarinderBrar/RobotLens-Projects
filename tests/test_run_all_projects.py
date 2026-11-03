from pathlib import Path
import json
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import run_all_projects as batch
from quickstart_processes import OwnedProcesses


class BatchRunnerTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="project batch ")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)

    def project(self, name, passed=True):
        script = self.root / name / "scripts/quickstart.py"
        script.parent.mkdir(parents=True)
        script.write_text(
            'import json\nfrom pathlib import Path\n'
            'report = Path("evidence/quickstart/report.json")\n'
            'report.parent.mkdir(parents=True, exist_ok=True)\n'
            f'report.write_text(json.dumps({{"passed": {passed}}}))\n'
            f'raise SystemExit({0 if passed else 1})\n')
        return script

    def test_discovery_and_selection_work_in_moved_folder(self):
        first = self.project("first")
        self.project("second")
        self.assertEqual(batch.discover_projects(self.root, ["first"]), [first])
        self.assertEqual(len(batch.discover_projects(self.root, [])), 2)

    def test_unknown_project_fails_instead_of_silently_skipping(self):
        self.project("first")
        with self.assertRaisesRegex(ValueError, "missing"):
            batch.discover_projects(self.root, ["missing"])

    def run_fixture(self, script):
        output = self.root / "reports"
        output.mkdir()
        sessions = self.root / "sessions"
        sessions.mkdir()
        return batch.run_project(script, output, sys.executable, 10, lambda: [], lambda: sessions)

    def test_failed_project_preserves_failure_and_report(self):
        result = self.run_fixture(self.project("failed", passed=False))
        self.assertEqual(result["result"], "FAIL")
        self.assertEqual(result["exit_code"], 1)
        self.assertTrue((self.root / "reports/failed/report.json").is_file())

    def test_stale_success_report_cannot_turn_crashed_script_into_pass(self):
        script = self.project("stale")
        report = script.parent.parent / "evidence/quickstart/report.json"
        report.parent.mkdir(parents=True)
        report.write_text('{"passed": true}')
        script.write_text('raise SystemExit(0)\n')
        result = self.run_fixture(script)
        self.assertEqual(result["result"], "FAIL")

    def test_success_requires_fresh_report_and_clean_shutdown(self):
        result = self.run_fixture(self.project("passed"))
        self.assertEqual(result["result"], "PASS")
        self.assertTrue(result["cleanup_ok"])

    def test_reused_pid_is_never_signalled(self):
        owned = OwnedProcesses()
        owned.identities = {123: 10}
        with patch("quickstart_processes.process_snapshot", return_value={123: (1, 20, "S")}), \
                patch("quickstart_processes.os.kill") as kill:
            owned.signal([123], 15)
        kill.assert_not_called()

    def main_fixture(self, arguments):
        client = SimpleNamespace(live_sessions=lambda: [], session_directory=lambda: self.root / "sessions")
        orchestrator = SimpleNamespace(become_child_subreaper=lambda: True)
        with patch.object(batch, "ROOT", self.root), patch.dict(
                sys.modules, {"mcp_client": client, "orchestrate": orchestrator}):
            return batch.main(arguments)

    def test_batch_continues_and_returns_failure_status(self):
        self.project("first", passed=False)
        self.project("second")
        output = self.root / "output"
        code = self.main_fixture(["--report-dir", str(output)])
        self.assertEqual(code, 1)
        report = json.loads((output / "summary.json").read_text())
        self.assertEqual(report["completed"], 2)
        self.assertEqual(report["counts"]["FAIL"], 1)
        self.assertEqual(report["counts"]["PASS"], 1)

    def test_existing_reports_are_never_overwritten(self):
        self.project("first")
        output = self.root / "output"
        output.mkdir()
        summary = output / "summary.json"
        summary.write_text("existing evidence")
        self.assertEqual(self.main_fixture(["--report-dir", str(output)]), 2)
        self.assertEqual(summary.read_text(), "existing evidence")


if __name__ == "__main__":
    unittest.main()
