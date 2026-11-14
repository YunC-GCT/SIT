"""Regression checks for transient Windows replacement locks and CLI exit."""

import ctypes
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import main
from core.batch import RunBatch
from core.atomic_json import JsonSaveError
from core.contracts import make_result, write_result
from core.orchestrator import PipelineOrchestrator
from tests import test_top_level as fixtures


class BatchPersistenceTests(unittest.TestCase):
    def test_permanent_lock_is_bounded_and_preserves_original_and_candidate(self):
        with tempfile.TemporaryDirectory() as tmp:
            batch = RunBatch(tmp).start()
            original = Path(batch.manifest_path).read_bytes()
            with patch("os.replace", side_effect=PermissionError("locked")) as replace, patch(
                "core.atomic_json.time.sleep",
            ) as sleep:
                with self.assertRaises(JsonSaveError) as caught:
                    batch.set_workflow(new_map=True)
            self.assertEqual(replace.call_count, 7)
            self.assertEqual(sleep.call_count, 6)
            self.assertEqual(Path(batch.manifest_path).read_bytes(), original)
            candidate = Path(caught.exception.candidate)
            self.assertTrue(json.loads(candidate.read_text(encoding="utf-8"))["workflow"]["new_map"])

    def test_result_json_uses_same_retry_and_removes_successful_temporary(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp, "result.json")
            result = make_result("example", "test", "success")
            replace = os.replace
            sources = []

            def denied_once(source, destination):
                sources.append(source)
                if len(sources) == 1:
                    raise PermissionError("locked")
                replace(source, destination)

            with patch("os.replace", side_effect=denied_once), patch("core.atomic_json.time.sleep"):
                write_result(str(path), result)
            self.assertEqual(json.loads(path.read_text(encoding="utf-8"))["status"], "success")
            self.assertFalse(Path(sources[0]).exists())
            self.assertEqual(len(list(Path(tmp).iterdir())), 1)

    def test_one_transient_replace_denial_does_not_abort_batch(self):
        with tempfile.TemporaryDirectory() as tmp:
            batch = RunBatch(tmp).start()
            replace = os.replace
            calls = []

            def denied_once(source, destination):
                calls.append((source, destination))
                if len(calls) == 1:
                    raise PermissionError(13, "temporary replacement lock", destination)
                return replace(source, destination)

            with patch("os.replace", side_effect=denied_once):
                batch.record_event({"type": "change_map_complete"})
            stored = json.loads(Path(batch.manifest_path).read_text(encoding="utf-8"))
            self.assertEqual(stored["events"][-1]["type"], "change_map_complete")
            self.assertGreaterEqual(len(calls), 2)

    def test_transient_manifest_lock_after_map_change_returns_to_completion_menu(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = fixtures.FakeModuleRunner()
            answers = iter(["m", "1", "q"])
            orchestrator = PipelineOrchestrator(
                "tests/fixtures/01_scattered.png", start=(0, 0), goal=(2, 2),
                drive=False, interactive_menu=True, run_root=tmp, runner=runner,
                input_fn=lambda _: next(answers), output_fn=lambda _: None,
            )
            replace = os.replace
            denied = []

            def deny_final_save_once(source, destination):
                if len(runner.commands) == 6 and str(destination) == orchestrator.batch.manifest_path and not denied:
                    denied.append(destination)
                    raise PermissionError("brief manifest lock after map change")
                return replace(source, destination)

            with patch("core.orchestrator.display_available", return_value=True), patch(
                "os.replace", side_effect=deny_final_save_once,
            ), patch("core.atomic_json.time.sleep"):
                report = orchestrator.run()
            self.assertEqual(report.status, "success", report.errors)
            self.assertEqual(len(denied), 1)
            self.assertEqual(len(runner.commands), 6)
            self.assertFalse(report.motion_started)
            self.assertEqual(orchestrator.batch.reload().manifest["status"], "success")

    @unittest.skipUnless(os.name == "nt", "Windows file sharing semantics")
    def test_real_windows_handle_denies_delete_until_retry(self):
        import threading
        with tempfile.TemporaryDirectory() as tmp:
            batch = RunBatch(tmp).start()
            kernel = ctypes.WinDLL("kernel32", use_last_error=True)
            create = kernel.CreateFileW
            create.argtypes = [ctypes.c_wchar_p, ctypes.c_uint32, ctypes.c_uint32,
                               ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p]
            create.restype = ctypes.c_void_p
            close = kernel.CloseHandle
            close.argtypes = [ctypes.c_void_p]
            close.restype = ctypes.c_int
            # Read handle permits other reads/writes but temporarily denies delete/rename.
            handle = create(batch.manifest_path, 0x80000000, 3, None, 3, 0, None)
            self.assertNotEqual(handle, ctypes.c_void_p(-1).value)
            release = threading.Timer(0.08, lambda: close(handle))
            release.start()
            try:
                batch.set_workflow(map_changed=True)
                self.assertTrue(batch.reload().manifest["workflow"]["map_changed"])
            finally:
                release.join()

    def test_ctrl_c_during_initial_map_selection_returns_130_without_traceback(self):
        with patch("main.interactive_terminal", return_value=True), patch(
            "main.display_available", return_value=True,
        ), patch("main._choose_image", side_effect=KeyboardInterrupt), patch(
            "main.run_pipeline",
        ) as run, patch("sys.stdout", new_callable=io.StringIO) as output:
            self.assertEqual(main.main(["--no-drive"]), 130)
            self.assertIn("取消", output.getvalue())
            run.assert_not_called()

    def test_persistent_save_failure_reports_path_without_retrying_error_recording(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = fixtures.FakeModuleRunner()
            calls = []

            def answers(prompt):
                calls.append(prompt)
                return "q"

            orchestrator = PipelineOrchestrator(
                "map.png", start=(0, 0), goal=(2, 2), drive=False,
                interactive_menu=True, run_root=tmp, runner=runner, input_fn=answers,
            )
            # Fail after successful module execution, exactly at final manifest writes.
            original_update = orchestrator.batch.update_stage

            def update(module, *args, **kwargs):
                if module == "motion_control":
                    raise JsonSaveError(orchestrator.batch.manifest_path, "candidate.tmp", PermissionError("locked"))
                return original_update(module, *args, **kwargs)

            with patch("core.orchestrator.display_available", return_value=True), patch.object(
                orchestrator.batch, "update_stage", side_effect=update,
            ):
                report = orchestrator.run()
            self.assertEqual(report.status, "failed")
            self.assertEqual(report.stage, "save_json")
            self.assertEqual(report.errors[0]["input"], orchestrator.batch.manifest_path)
            self.assertEqual(report.errors[0]["code"], "JSON_SAVE_FAILED")
            self.assertEqual(calls, [])
            self.assertEqual(len(runner.commands), 3)


if __name__ == "__main__":
    unittest.main()
