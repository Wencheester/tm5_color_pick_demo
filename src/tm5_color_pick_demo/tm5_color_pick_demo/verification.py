from tm5_color_pick_demo.grid_mapping import normalize_grid_cell


def verify_targets(after_state, target_state):
    failures = []
    normalized_after = {
        color: normalize_grid_cell(cell) for color, cell in after_state.items()
    }
    normalized_target = {
        color: normalize_grid_cell(cell) for color, cell in target_state.items()
    }

    for color, expected in sorted(normalized_target.items()):
        actual = normalized_after.get(color)
        if actual != expected:
            failures.append(
                {
                    "color": color,
                    "expected": expected,
                    "actual": actual,
                }
            )

    return len(failures) == 0, failures


def format_failures(failures):
    if not failures:
        return "verification succeeded"
    return "; ".join(
        "%s expected %s, got %s"
        % (failure["color"], failure["expected"], failure["actual"])
        for failure in failures
    )
