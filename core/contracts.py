"""Cross-module JSON contracts.

Implementation modules: contracts_types, contracts_payloads, contracts_io.
The public names remain importable from ``core.contracts``.
"""

from __future__ import annotations

from .contracts_io import (
    _main,
    make_error,
    make_result,
    read_result,
    require_success,
    validate_error,
    validate_result,
    write_result,
)
from .contracts_payloads import (
    array_payload,
    grid_map_payload,
    metric_payload,
    module_entry,
    path_payload,
    path_segment_payload,
)
from .contracts_types import (
    ContractError,
    ROW_COL_COORDINATE_SYSTEM,
    SCHEMA_VERSION,
    STATUSES,
    STATUS_FAILED,
    STATUS_PENDING,
    STATUS_RUNNING,
    STATUS_SUCCESS,
    utc_now,
)

__all__ = [
    "ContractError",
    "ROW_COL_COORDINATE_SYSTEM",
    "SCHEMA_VERSION",
    "STATUSES",
    "STATUS_FAILED",
    "STATUS_PENDING",
    "STATUS_RUNNING",
    "STATUS_SUCCESS",
    "array_payload",
    "grid_map_payload",
    "make_error",
    "make_result",
    "metric_payload",
    "module_entry",
    "path_payload",
    "path_segment_payload",
    "read_result",
    "require_success",
    "utc_now",
    "validate_error",
    "validate_result",
    "write_result",
]

if __name__ == "__main__":
    raise SystemExit(_main())
