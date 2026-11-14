"""Shared terminal map chooser for startup, map changes and recovery."""

from __future__ import annotations

import os
from typing import Callable


IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp")
DEFAULT_MAP_DIRECTORY = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "map_pipeline", "data", "input",
)


def choose_map(
    *,
    current: str | None = None,
    directory: str = DEFAULT_MAP_DIRECTORY,
    input_fn: Callable[[str], str] | None = None,
    output_fn: Callable[[str], None] = print,
) -> str | None:
    """Return a validated image path, or None on explicit cancellation.

    Invalid input stays in the chooser. No caller state is changed until a
    selection is returned. Empty input selects the current map or first image.
    Image decoding remains the map module's responsibility.
    """
    prompt = input_fn if input_fn is not None else input
    files = sorted(
        name for name in os.listdir(directory)
        if name.lower().endswith(IMAGE_EXTENSIONS)
        and os.path.isfile(os.path.join(directory, name))
    ) if os.path.isdir(directory) else []
    output_fn("可用地图 / Available maps:")
    for index, name in enumerate(files, 1):
        output_fn(f"  [{index}] {name}")
    if current:
        output_fn(f"当前地图：{current}")
    if not files:
        output_fn("没有内置地图，请输入图片路径。")
    default = "保留当前地图" if current else "使用第一张"
    while True:
        value = prompt(
            f"输入地图编号或图片路径（回车{default}，b 取消）："
        ).strip().strip('"').strip("'")
        if value.lower() == "b":
            return None
        if not value:
            path = current or (os.path.join(directory, files[0]) if files else None)
            if path is None:
                output_fn("请输入图片路径，或输入 b 取消。")
                continue
        elif value.isdecimal():
            index = int(value)
            if not 1 <= index <= len(files):
                output_fn("地图编号无效，请使用列表中的编号或输入图片路径。")
                continue
            path = os.path.join(directory, files[index - 1])
        else:
            path = os.path.expanduser(value)
        path = os.path.abspath(path)
        if not os.path.isfile(path):
            output_fn(f"图片不存在：{path}；请重新选择。")
            continue
        if not path.lower().endswith(IMAGE_EXTENSIONS):
            output_fn("请选择 .png、.jpg、.jpeg 或 .bmp 图片。")
            continue
        output_fn(f"已选择：{os.path.basename(path)}")
        return path
