"""基于规划器公共目录的终端算法选择与校验。"""

from typing import Callable, Sequence

from path_planner.catalog import algorithm_catalog


def choose_algorithms(
    *,
    current: Sequence[str] = (),
    config_path: str | None = None,
    input_fn: Callable[[str], str] = input,
    output_fn: Callable[[str], None] = print,
) -> list[str] | None:
    """交互选择已注册算法，返回名称列表；取消返回 None，非法输入继续提示。"""
    catalog, default = algorithm_catalog(config_path)
    names = [item["name"] for item in catalog]
    output_fn("可用规划算法：")
    for index, item in enumerate(catalog, 1):
        output_fn(f"  [{index}] {item['label']}  ({item['name']})")
    output_fn(f"当前选择：{', '.join(current) if current else default}")
    while True:
        raw = input_fn(
            "输入算法编号或名称（多个用逗号分隔；回车保留，b 取消）："
        ).strip().lower()
        if raw in {"", "b"}:
            return None
        tokens = raw.replace("，", ",").split(",")
        selected = []
        for token in tokens:
            name = token.strip()
            if name.isdecimal():
                number = int(name)
                name = names[number - 1] if 1 <= number <= len(names) else ""
            if name not in names:
                output_fn("算法选项无效，请输入列表中的编号或名称；换地图请先 b 返回，再输入 m。")
                break
            if name not in selected:
                selected.append(name)
        else:
            output_fn("已选择算法：" + ", ".join(selected))
            return selected
