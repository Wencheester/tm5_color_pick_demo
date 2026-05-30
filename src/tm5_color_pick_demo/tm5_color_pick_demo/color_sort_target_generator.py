COLOR_SORT_TARGET_STATE = {
    "yellow": "right_top",
    "blue": "right_middle",
    "red": "right_bottom",
}


def generate_color_sort_target_state():
    return dict(COLOR_SORT_TARGET_STATE)


def target_state_to_goals(target_state):
    return [
        {"cube": cube, "target": target}
        for cube, target in target_state.items()
    ]
