CAMERA_TO_LOGIC = {
    "left_top": "right_top",
    "center_top": "right_middle",
    "right_top": "right_bottom",
    "left_middle": "center_top",
    "center_middle": "center_middle",
    "right_middle": "center_bottom",
    "left_bottom": "left_top",
    "center_bottom": "left_middle",
    "right_bottom": "left_bottom",
}

LOGIC_TO_CAMERA = {logic: camera for camera, logic in CAMERA_TO_LOGIC.items()}

CHINESE_TO_CELL = {
    "左上": "left_top",
    "中上": "center_top",
    "右上": "right_top",
    "左中": "left_middle",
    "中中": "center_middle",
    "中间": "center_middle",
    "中心": "center_middle",
    "右中": "right_middle",
    "左下": "left_bottom",
    "中下": "center_bottom",
    "右下": "right_bottom",
}

CELL_TO_CHINESE = {
    "left_top": "左上",
    "center_top": "中上",
    "right_top": "右上",
    "left_middle": "左中",
    "center_middle": "中中",
    "right_middle": "右中",
    "left_bottom": "左下",
    "center_bottom": "中下",
    "right_bottom": "右下",
}


def normalize_grid_cell(cell):
    value = str(cell).strip().lower()
    if value in CHINESE_TO_CELL:
        return CHINESE_TO_CELL[value]
    return value.replace(" ", "_")


def camera_cell_to_logic_cell(camera_cell):
    normalized = normalize_grid_cell(camera_cell)
    if normalized not in CAMERA_TO_LOGIC:
        raise ValueError("Unknown camera grid cell '%s'." % camera_cell)
    return CAMERA_TO_LOGIC[normalized]


def logic_cell_to_camera_cell(logic_cell):
    normalized = normalize_grid_cell(logic_cell)
    if normalized not in LOGIC_TO_CAMERA:
        raise ValueError("Unknown logic grid cell '%s'." % logic_cell)
    return LOGIC_TO_CAMERA[normalized]


def camera_state_to_logic_state(camera_state):
    return {
        color: camera_cell_to_logic_cell(cell)
        for color, cell in camera_state.items()
    }


def format_state_chinese(state):
    return {
        color: CELL_TO_CHINESE.get(normalize_grid_cell(cell), str(cell))
        for color, cell in state.items()
    }
