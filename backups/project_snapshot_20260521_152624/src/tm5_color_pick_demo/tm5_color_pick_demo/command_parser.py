COLOR_ALIASES = {
    "红色": "red",
    "红": "red",
    "red": "red",
    "黄色": "yellow",
    "黄": "yellow",
    "yellow": "yellow",
    "蓝色": "blue",
    "蓝": "blue",
    "blue": "blue",
}

CELL_ALIASES = {
    "左上": "left_top",
    "上左": "left_top",
    "left top": "left_top",
    "left_top": "left_top",
    "中上": "center_top",
    "上中": "center_top",
    "center top": "center_top",
    "center_top": "center_top",
    "右上": "right_top",
    "上右": "right_top",
    "right top": "right_top",
    "right_top": "right_top",
    "左中": "left_middle",
    "中左": "left_middle",
    "left middle": "left_middle",
    "left_middle": "left_middle",
    "中间": "center_middle",
    "中心": "center_middle",
    "正中": "center_middle",
    "center middle": "center_middle",
    "center_middle": "center_middle",
    "右中": "right_middle",
    "中右": "right_middle",
    "right middle": "right_middle",
    "right_middle": "right_middle",
    "左下": "left_bottom",
    "下左": "left_bottom",
    "left bottom": "left_bottom",
    "left_bottom": "left_bottom",
    "中下": "center_bottom",
    "下中": "center_bottom",
    "center bottom": "center_bottom",
    "center_bottom": "center_bottom",
    "右下": "right_bottom",
    "下右": "right_bottom",
    "right bottom": "right_bottom",
    "right_bottom": "right_bottom",
}


def parse_command(command):
    text = command.strip().lower()
    cube = _find_alias(text, COLOR_ALIASES)
    if cube is None:
        raise ValueError("Could not parse cube color from command: %s" % command)

    place_cell = _find_alias(text, CELL_ALIASES)
    if place_cell is None:
        raise ValueError("Could not parse target cell from command: %s" % command)

    return cube, place_cell


def _find_alias(text, aliases):
    for alias in sorted(aliases, key=len, reverse=True):
        if alias in text:
            return aliases[alias]
    return None
