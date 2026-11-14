"""公共契约常量、异常及基础辅助函数。"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

SCHEMA_VERSION = "1.0"
STATUS_PENDING = "pending"
STATUS_RUNNING = "running"
STATUS_SUCCESS = "success"
STATUS_FAILED = "failed"
STATUSES = {STATUS_PENDING, STATUS_RUNNING, STATUS_SUCCESS, STATUS_FAILED}
ROW_COL_COORDINATE_SYSTEM = {
    "axes": ["row", "col"],
    "row": "increases_south",
    "col": "increases_east",
    "unit": "grid_cell",
}

class ContractError(ValueError):
    """输入或输出不符合统一契约。"""

def utc_now() -> str:
    """返回带时区的 UTC 时间字符串，供结果和清单统一记录时间。"""
    return datetime.now(timezone.utc).isoformat()

def _json_ready(value: Any) -> Any:
    """递归转换数组、标量和容器，使其适合 JSON 序列化。"""
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, tuple):
        return [_json_ready(item) for item in value]
    if isinstance(value, list):
        return [_json_ready(item) for item in value]
    if isinstance(value, dict):
        return {key: _json_ready(item) for key, item in value.items()}
    return value

def _shape_of(value: Any) -> list[int]:
    """递归推断嵌套列表的形状，非规则数组抛出契约错误。"""
    if hasattr(value, "shape"):
        return [int(part) for part in value.shape]
    if isinstance(value, (list, tuple)):
        length = len(value)
        if length == 0:
            return [0]
        child_shape = _shape_of(value[0])
        for item in value[1:]:
            if _shape_of(item) != child_shape:
                raise ContractError("数组必须是规则矩形结构")
        return [length] + child_shape
    return []


__all__ = [
    "ContractError",
    "ROW_COL_COORDINATE_SYSTEM",
    "SCHEMA_VERSION",
    "STATUSES",
    "STATUS_FAILED",
    "STATUS_PENDING",
    "STATUS_RUNNING",
    "STATUS_SUCCESS",
    "_json_ready",
    "_shape_of",
    "utc_now",
]
