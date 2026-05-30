import itertools


VALID_CUBES = ("red", "yellow", "blue")
GRID_CELLS = (
    "left_top",
    "center_top",
    "right_top",
    "left_middle",
    "center_middle",
    "right_middle",
    "left_bottom",
    "center_bottom",
    "right_bottom",
)

CELL_COORDS = {
    "left_top": (0, 0),
    "center_top": (0, 1),
    "right_top": (0, 2),
    "left_middle": (1, 0),
    "center_middle": (1, 1),
    "right_middle": (1, 2),
    "left_bottom": (2, 0),
    "center_bottom": (2, 1),
    "right_bottom": (2, 2),
}

AREAS = {
    "left": ("left_top", "left_middle", "left_bottom"),
    "center": ("center_top", "center_middle", "center_bottom"),
    "middle": ("center_top", "center_middle", "center_bottom"),
    "right": ("right_top", "right_middle", "right_bottom"),
    "top": ("left_top", "center_top", "right_top"),
    "upper": ("left_top", "center_top", "right_top"),
    "bottom": ("left_bottom", "center_bottom", "right_bottom"),
    "lower": ("left_bottom", "center_bottom", "right_bottom"),
    "center_area": ("center_middle",),
}


class PlanningError(RuntimeError):
    pass


def plan_actions(current_state, goal_constraints):
    state = _validate_state(current_state)
    goals = _normalize_goals(state, goal_constraints)
    target_state = _assign_targets(state, goals)
    action_plan = _make_action_plan(state, target_state)
    return target_state, action_plan


def _validate_state(current_state):
    if not isinstance(current_state, dict):
        raise PlanningError("current_state must be a mapping.")

    state = {}
    used_cells = set()
    for cube in VALID_CUBES:
        cell = str(current_state.get(cube, "")).strip()
        if cell not in CELL_COORDS:
            raise PlanningError("Cube %s has invalid cell '%s'." % (cube, cell))
        if cell in used_cells:
            raise PlanningError("Cell '%s' is occupied by more than one cube." % cell)
        used_cells.add(cell)
        state[cube] = cell
    return state


def _normalize_goals(current_state, goal_constraints):
    if not isinstance(goal_constraints, list) or not goal_constraints:
        raise PlanningError("goal_constraints must be a non-empty list.")

    goals_by_cube = {}
    for index, goal in enumerate(goal_constraints, start=1):
        if not isinstance(goal, dict):
            raise PlanningError("Goal %d must be a mapping." % index)

        cube = str(goal.get("cube", "")).strip().lower()
        if cube not in VALID_CUBES:
            raise PlanningError("Goal %d has invalid cube '%s'." % (index, cube))

        target_cube = str(goal.get("target_cube_position", "")).strip().lower()
        if target_cube:
            if target_cube not in VALID_CUBES:
                raise PlanningError(
                    "Goal %d has invalid target cube '%s'." % (index, target_cube)
                )
            candidates = (current_state[target_cube],)
        else:
            target = str(goal.get("target", goal.get("place_cell", ""))).strip().lower()
            candidates = _target_candidates(target, index)

        goals_by_cube[cube] = {
            "cube": cube,
            "candidates": candidates,
            "source": dict(goal),
        }

    return list(goals_by_cube.values())


def _target_candidates(target, index):
    normalized = target.replace(" ", "_")
    if normalized in CELL_COORDS:
        return (normalized,)
    if normalized in AREAS:
        return AREAS[normalized]
    raise PlanningError("Goal %d has invalid target '%s'." % (index, target))


def _assign_targets(current_state, goals):
    best_assignment = None
    best_cost = None
    candidate_lists = [goal["candidates"] for goal in goals]
    goal_cubes = {goal["cube"] for goal in goals}

    for cells in itertools.product(*candidate_lists):
        if len(set(cells)) != len(cells):
            continue
        assignment = {
            goal["cube"]: cell for goal, cell in zip(goals, cells)
        }
        cost = sum(
            _assignment_cost(current_state, goal_cubes, cube, cell)
            for cube, cell in assignment.items()
        )
        if best_cost is None or cost < best_cost:
            best_cost = cost
            best_assignment = assignment

    if best_assignment is None:
        raise PlanningError("Could not assign unique targets for requested goals.")
    return best_assignment


def _assignment_cost(current_state, goal_cubes, cube, cell):
    blocker = _cube_at_cell(current_state, cell)
    unrelated_blocker_cost = 1000 if blocker is not None and blocker not in goal_cubes else 0
    moving_target_cost = 10 if current_state[cube] != cell else 0
    return (
        unrelated_blocker_cost
        + moving_target_cost
        + _cell_distance(current_state[cube], cell)
    )


def _make_action_plan(current_state, target_state):
    state = dict(current_state)
    actions = []

    for cube in sorted(target_state):
        _ensure_cube_at_target(cube, state, target_state, actions, [])

    return actions


def _ensure_cube_at_target(cube, state, target_state, actions, stack):
    target = target_state.get(cube)
    if target is None or state[cube] == target:
        return

    blocker = _cube_at_cell(state, target)
    if blocker is not None and blocker != cube:
        if blocker in stack:
            buffer_cell = _choose_buffer_cell(blocker, state, target_state)
            _append_move(
                actions,
                state,
                blocker,
                buffer_cell,
                "break cycle with buffer",
            )
        elif blocker in target_state:
            _ensure_cube_at_target(blocker, state, target_state, actions, stack + [cube])
        else:
            buffer_cell = _choose_buffer_cell(blocker, state, target_state)
            _append_move(
                actions,
                state,
                blocker,
                buffer_cell,
                "clear occupied target",
            )

    blocker = _cube_at_cell(state, target)
    if blocker is not None and blocker != cube:
        raise PlanningError(
            "Could not clear target cell '%s' for cube '%s'." % (target, cube)
        )

    _append_move(actions, state, cube, target, "requested goal")


def _append_move(actions, state, cube, target, reason):
    source = state[cube]
    if source == target:
        return
    actions.append(
        {
            "cube": cube,
            "from": source,
            "to": target,
            "reason": reason,
        }
    )
    state[cube] = target


def _choose_buffer_cell(cube, state, target_state):
    occupied = set(state.values())
    reserved_targets = set(target_state.values())
    candidates = [
        cell for cell in GRID_CELLS
        if cell not in occupied and cell not in reserved_targets
    ]
    if not candidates:
        candidates = [cell for cell in GRID_CELLS if cell not in occupied]
    if not candidates:
        raise PlanningError("No buffer cell is available.")
    return min(candidates, key=lambda cell: _cell_distance(state[cube], cell))


def _cube_at_cell(state, cell):
    for cube, cube_cell in state.items():
        if cube_cell == cell:
            return cube
    return None


def _cell_distance(left, right):
    left_row, left_col = CELL_COORDS[left]
    right_row, right_col = CELL_COORDS[right]
    return abs(left_row - right_row) + abs(left_col - right_col)
