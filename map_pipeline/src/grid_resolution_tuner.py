"""
终端交互调整栅格宽度
"""

def tune_resolution(default_width: int, img_width: int, img_height: int,
                    preview_func) -> int:
    """
    终端交互调整栅格宽度。先告知用户当前宽高,然后让用户输入新宽度,
    直接回车则保持当前宽度(进入下一步)。

    :param default_width: 默认宽度（列数）
    :param img_width: 原始图片像素宽度
    :param img_height: 原始图片像素高度
    :param preview_func: 函数，接受目标宽度，返回 (rows, cols)
    :return: 最终确认的宽度
    """
    current_width = default_width
    invalid_attempts = 0
    MAX_INVALID = 5
    while True:
        rows, cols = preview_func(current_width)
        print(f"\n当前栅格尺寸: {rows} 行 × {cols} 列 (宽度={current_width} 列)")
        user_input = input(
            "输入新栅格宽度（正整数,直接回车保持当前进入下一步,q=退出）: "
        ).strip()
        if user_input == "":
            print(f"已确认栅格宽度: {current_width}")
            return current_width
        if user_input.lower() in ('q', 'quit', 'exit'):
            raise SystemExit("用户在调宽步骤主动退出。")
        try:
            new_width = int(user_input)
            if new_width < 1:
                print("宽度必须为正整数,请重新输入(直接回车保持当前,q=退出)。")
                invalid_attempts += 1
            else:
                current_width = new_width
                invalid_attempts = 0  # 成功一次就清零
        except ValueError:
            print("输入无效,请输入正整数(直接回车保持当前,q=退出)。")
            invalid_attempts += 1

        if invalid_attempts >= MAX_INVALID:
            raise SystemExit(
                f"连续 {MAX_INVALID} 次无效输入,自动退出调宽步骤。"
                f"当前栅格宽度仍为 {current_width}。"
            )