import matplotlib
matplotlib.use('Agg')  # 非交互式后端，直接保存图片
import matplotlib.pyplot as plt
import numpy as np

def draw_single_path(grid, path, start, goal, filepath, draw_grid=True):
    """
    绘制单条路径，可选绘制栅格线。
    """
    rows, cols = grid.shape
    fig, ax = plt.subplots(figsize=(8, 8))

    # 使用 'gray_r'：0 显示白色，1 显示黑色
    ax.imshow(grid, cmap='gray_r', origin='upper')

    # 绘制栅格线
    if draw_grid:
        for i in range(1, rows):
            ax.axhline(y=i - 0.5, color='gray', linewidth=0.5)
        for j in range(1, cols):
            ax.axvline(x=j - 0.5, color='gray', linewidth=0.5)

    # 绘制路径（青色线）
    if path:
        path_arr = np.array(path)
        ax.plot(path_arr[:, 1], path_arr[:, 0], color='cyan', linewidth=2,
                marker='o', markersize=3, label='Path')

    # 起点和终点
    ax.scatter(start[1], start[0], c='green', marker='o', s=100, label='Start')
    ax.scatter(goal[1], goal[0], c='red', marker='X', s=100, label='Goal')

    ax.legend()
    ax.set_title("Planned Path with Grid")
    plt.tight_layout()
    plt.savefig(filepath, dpi=150)
    plt.close()


def draw_comparison(grid, results, start, goal, filepath, draw_grid=True):
    """
    绘制多条路径对比图，可选绘制栅格线。
    """
    rows, cols = grid.shape
    fig, ax = plt.subplots(figsize=(8, 8))

    ax.imshow(grid, cmap='gray_r', origin='upper')

    if draw_grid:
        for i in range(1, rows):
            ax.axhline(y=i - 0.5, color='gray', linewidth=0.5)
        for j in range(1, cols):
            ax.axvline(x=j - 0.5, color='gray', linewidth=0.5)

    colors = ['cyan', 'magenta', 'yellow', 'orange', 'lime']
    for i, (name, data) in enumerate(results.items()):
        path = data.get('path', [])
        if path:
            path_arr = np.array(path)
            ax.plot(path_arr[:, 1], path_arr[:, 0], color=colors[i % len(colors)],
                    linewidth=2, label=name)

    ax.scatter(start[1], start[0], c='green', marker='o', s=100)
    ax.scatter(goal[1], goal[0], c='red', marker='X', s=100)
    ax.legend()
    ax.set_title("Algorithm Comparison with Grid")
    plt.tight_layout()
    plt.savefig(filepath, dpi=150)
    plt.close()