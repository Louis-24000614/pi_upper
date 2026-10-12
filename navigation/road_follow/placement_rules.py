"""比赛规则第 9 页图 7：蓝色 20 段可放障碍/涵洞，橙色 2 段仅涵洞。"""


_BLUE_SEGMENTS = frozenset(
    frozenset((f"{row}_{left}", f"{row}_{left+1}"))
    for row in range(2, 6) for left in (1, 3)
) | frozenset(
    frozenset((f"{row}_{column}", f"{row+1}_{column}"))
    for row in range(2, 5) for column in range(1, 5)
)
_CULVERT_ONLY_SEGMENTS = frozenset((
    frozenset(("1_2", "2_2")), frozenset(("1_3", "2_3")),
))
_ALLOWED_SEGMENTS = {
    "obstacle": _BLUE_SEGMENTS,
    "culvert": _BLUE_SEGMENTS | _CULVERT_ONLY_SEGMENTS,
}


def placement_allowed(edge, kind):
    """按实际目标边的两端判定；双向一致，隧道和未知边不放行。"""
    return (edge is not None and not edge.tunnel
            and frozenset((edge.u, edge.v)) in _ALLOWED_SEGMENTS[kind])
