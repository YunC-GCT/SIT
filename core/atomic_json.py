"""支持 Windows 短暂文件锁重试的原子 JSON 持久化。"""

from __future__ import annotations

import json
import os
import tempfile
import time


class JsonSaveError(OSError):
    """目标文件替换失败时保留完整候选文件，便于恢复。"""

    def __init__(self, path: str, candidate: str, reason: OSError):
        """记录目标路径、完整候选文件和底层错误，供保存失败后恢复。"""
        self.path = path
        self.candidate = candidate
        super().__init__(f"无法保存 JSON：{path}；待保存文件：{candidate}；原因：{reason}")


def write_json_atomic(path: str, value: dict) -> None:
    """先写同目录的唯一临时文件，再原子替换目标 JSON。

    遇到 PermissionError 共等待至多 1.25 秒重试；替换失败时保留完整候选文件。
    不会先截断原文件，也不负责串行化多个进程对清单的并发修改。"""
    path = os.path.abspath(path)
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    fd, candidate = tempfile.mkstemp(prefix=f".{os.path.basename(path)}-", suffix=".tmp", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(value, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
    except BaseException:
        os.unlink(candidate)
        raise
    delays = (0.05, 0.1, 0.2, 0.3, 0.3, 0.3)
    for attempt in range(len(delays) + 1):
        try:
            os.replace(candidate, path)
            return
        except PermissionError as exc:
            if attempt == len(delays):
                raise JsonSaveError(path, candidate, exc) from exc
            time.sleep(delays[attempt])
        except OSError as exc:
            raise JsonSaveError(path, candidate, exc) from exc
