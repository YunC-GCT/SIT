"""Atomic JSON persistence tolerant of brief Windows replacement locks."""

from __future__ import annotations

import json
import os
import tempfile
import time


class JsonSaveError(OSError):
    """A complete candidate remains available when destination replacement fails."""

    def __init__(self, path: str, candidate: str, reason: OSError):
        self.path = path
        self.candidate = candidate
        super().__init__(f"无法保存 JSON：{path}；待保存文件：{candidate}；原因：{reason}")


def write_json_atomic(path: str, value: dict) -> None:
    """Replace from a unique sibling file, retrying PermissionError for 1.25s.

    Never truncate the destination. On replacement failure keep the complete
    candidate for recovery; this does not serialize concurrent manifest edits.
    """
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
