"""运行批次生命周期与 manifest 管理。"""

from __future__ import annotations

import json
import os
import platform
import uuid
from datetime import datetime, timezone
from typing import Any

from .atomic_json import write_json_atomic

from .contracts import (
    SCHEMA_VERSION,
    STATUSES,
    STATUS_FAILED,
    STATUS_PENDING,
    STATUS_RUNNING,
    STATUS_SUCCESS,
    ContractError,
    module_entry,
    validate_error,
)


class RunBatch:
    """每次实验的隔离目录。

    目录中的 ``manifest.json`` 记录阶段状态；阶段结果由各模块自己写入，
    顶层只登记路径和状态。
    """

    def __init__(self, root: str = "runs", run_id: str | None = None):
        self.root = os.path.abspath(root)
        self.run_id = run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8]
        self.path = os.path.join(self.root, self.run_id)
        self.manifest_path = os.path.join(self.path, "manifest.json")
        self.manifest = self._load_or_create_manifest()

    def _load_or_create_manifest(self) -> dict:
        if os.path.exists(self.manifest_path):
            with open(self.manifest_path, "r", encoding="utf-8") as handle:
                manifest = json.load(handle)
            if manifest.get("run_id") != self.run_id:
                raise ContractError("运行批次 manifest 的 run_id 不匹配")
            manifest.setdefault("modules", [])
            manifest.setdefault("stages", {})
            manifest.setdefault("artifacts", {})
            manifest.setdefault("events", [])
            manifest.setdefault("workflow", {})
            return manifest

        return {
            "schema_version": SCHEMA_VERSION,
            "run_id": self.run_id,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "updated_at": datetime.now(timezone.utc).isoformat(),
            "status": STATUS_RUNNING,
            "version": {
                "python": platform.python_version(),
                "platform": platform.platform(),
            },
            "modules": [],
            "stages": {},
            "artifacts": {},
            "events": [],
            "workflow": {},
        }

    def start(self) -> "RunBatch":
        os.makedirs(self.path, exist_ok=True)
        self.manifest["status"] = STATUS_RUNNING
        self.manifest.pop("finished_at", None)
        self._save()
        return self

    def stage_dir(self, module: str) -> str:
        directory = os.path.join(self.path, module)
        os.makedirs(directory, exist_ok=True)
        return directory

    def log_path(self, module: str, filename: str = "module.log") -> str:
        return os.path.join(self.stage_dir(module), filename)

    def update_stage(self, module: str, status: str, *, result: str | None = None,
                     error: Any = None) -> None:
        if status not in STATUSES:
            raise ContractError(f"未知阶段状态: {status}")
        normalized_error = self._validate_stage_error(status, error)
        existing = self.manifest["stages"].get(module)
        record = {
            "status": status,
            "updated_at": datetime.now(timezone.utc).isoformat(),
            "result": result,
            "error": normalized_error,
        }

        if existing and existing.get("status") == STATUS_SUCCESS and status == STATUS_FAILED:
            if not result or result == existing.get("result"):
                raise ContractError("失败阶段不能覆盖已有 success 阶段")
            self._append_stage_attempt(existing, record)
            self._save()
            return

        if existing and existing.get("status") in {STATUS_SUCCESS, STATUS_FAILED}:
            previous = {
                key: value
                for key, value in existing.items()
                if key not in {"attempts", "last_attempt"}
            }
            attempts = list(existing.get("attempts", []))
            attempts.append(previous)
            record["attempts"] = attempts

        self.manifest["stages"][module] = record
        self._upsert_module(module, status, result=result)
        self._save()

    @staticmethod
    def _append_stage_attempt(existing: dict, record: dict) -> None:
        attempts = list(existing.get("attempts", []))
        attempts.append(record)
        existing["attempts"] = attempts
        existing["last_attempt"] = record

    def finish(self, status: str = STATUS_SUCCESS) -> None:
        if status not in STATUSES:
            raise ContractError(f"未知批次状态: {status}")
        self.manifest["status"] = status
        self.manifest["finished_at"] = datetime.now(timezone.utc).isoformat()
        self._save()

    def register_module(self, module: str, *, status: str = STATUS_PENDING,
                        result: str | None = None) -> None:
        existing_stage = self.manifest["stages"].get(module)
        if (
            existing_stage
            and existing_stage.get("status") == STATUS_SUCCESS
            and status == STATUS_PENDING
        ):
            self._save()
            return
        self._upsert_module(module, status, result=result)
        self._save()

    def set_workflow(self, **values: Any) -> None:
        self.manifest.setdefault("workflow", {}).update(values)
        self._save()

    def record_artifact(self, name: str, path: str) -> None:
        self.manifest.setdefault("artifacts", {})[name] = path
        self._save()

    def record_event(self, event: dict[str, Any]) -> None:
        self.manifest.setdefault("events", []).append(event)
        self._save()

    def reload(self) -> "RunBatch":
        self.manifest = self._load_or_create_manifest()
        return self

    def _upsert_module(self, module: str, status: str, *, result: str | None = None) -> None:
        entry = module_entry(module, status, result_path=result)
        modules = [item for item in self.manifest["modules"] if item.get("name") != module]
        modules.append(entry)
        self.manifest["modules"] = modules

    def _validate_stage_error(self, status: str, error: Any) -> Any:
        if status != STATUS_FAILED:
            return error
        if error is None:
            raise ContractError("failed 阶段必须记录结构化错误")
        if isinstance(error, list):
            if not error:
                raise ContractError("failed 阶段必须记录结构化错误")
            return [validate_error(item) for item in error]
        return validate_error(error)

    def _save(self) -> None:
        os.makedirs(self.path, exist_ok=True)
        self.manifest["updated_at"] = datetime.now(timezone.utc).isoformat()
        write_json_atomic(self.manifest_path, self.manifest)

    def __enter__(self) -> "RunBatch":
        return self.start()

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.finish(STATUS_FAILED if exc else STATUS_SUCCESS)
        return False
