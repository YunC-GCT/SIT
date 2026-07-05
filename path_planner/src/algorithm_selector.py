def select_algorithms(available):
    print("可用的路径规划算法:")
    for i, name in enumerate(available, 1):
        print(f"  [{i}] {name}")
    print("  可多选（逗号分隔），或输入 'all' 全选")
    choice = input("请输入选择的算法: ").strip()
    if choice.lower() == 'all':
        return available
    indices = [x.strip() for x in choice.split(',')]
    selected = []
    for idx in indices:
        try:
            i = int(idx) - 1
            selected.append(available[i])
        except (ValueError, IndexError):
            print(f"  跳过无效序号: {idx}")
    return selected