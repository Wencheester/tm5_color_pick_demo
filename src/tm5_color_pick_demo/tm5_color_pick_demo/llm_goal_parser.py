import json
import urllib.error
import urllib.request

from tm5_color_pick_demo.task_planner import AREAS, CELL_COORDS, VALID_CUBES


SYSTEM_PROMPT = """You are a robot task planner command parser for a ROS 2 cube pick-and-place demo.
Return JSON only. Do not use markdown. Do not explain.

The planner can move red, yellow, and blue cubes on a 3x3 grid.
Legal cube values: red, yellow, blue.
Legal exact target values: left_top, left_middle, left_bottom, center_top, center_middle, center_bottom, right_top, right_middle, right_bottom.
Legal area target values: left, center, right, top, bottom.

Map colors from any language when clear:
red / 红 / 红色 / red cube -> red
yellow / 黄 / 黄色 / yellow cube -> yellow
blue / 蓝 / 蓝色 / blue cube -> blue

Map exact target cells from natural language when clear:
left top / top left / 左上 / 上左 -> left_top
left middle / middle left / 左中 / 中左 -> left_middle
left bottom / bottom left / 左下 / 下左 -> left_bottom
center top / top center / 中上 / 上中 -> center_top
center / middle / center middle / 中间 / 中心 / 正中 / 中间区域 -> center_middle
center bottom / bottom center / 中下 / 下中 -> center_bottom
right top / top right / 右上 / 上右 -> right_top
right middle / middle right / 右中 / 中右 -> right_middle
right bottom / bottom right / 右下 / 下右 -> right_bottom

Map area targets when the user asks for a region:
left side / 左边 / 左侧 -> left
center column / middle column / 中间列 / 中间一列 / 中列 -> center
right side / 右边 / 右侧 -> right
top row / 上面 / 上方 -> top
bottom row / 下面 / 下方 -> bottom

For swaps, express each cube target as the other cube's original position:
{"cube": "red", "target_cube_position": "yellow"}

If the user provides multiple simultaneous goals, output them in the same stage.
If the user uses sequencing words such as then / next / after / 然后 / 再 / 之后,
output multiple stages in order. Sequential swaps must use multiple stages.
Resolve pronouns such as it / that cube / 它 / 这个 / 那个 to the most recent explicit cube.
If a goal is missing cube or target, or asks for unsupported planning, output ok=false.

Single-stage schema:
{"ok": true, "goals": [{"cube": "yellow", "target": "right_top"}]}
Area schema:
{"ok": true, "goals": [{"cube": "yellow", "target": "right"}]}
Swap schema:
{"ok": true, "goals": [{"cube": "red", "target_cube_position": "yellow"}, {"cube": "yellow", "target_cube_position": "red"}]}
Multi-stage schema:
{"ok": true, "stages": [{"goals": [{"cube": "yellow", "target_cube_position": "blue"}, {"cube": "blue", "target_cube_position": "yellow"}]}, {"goals": [{"cube": "red", "target_cube_position": "blue"}, {"cube": "blue", "target_cube_position": "red"}]}]}
Failure schema:
{"ok": false, "error": "short reason"}
"""


class LlmGoalParser:
    def __init__(
        self,
        api_key,
        base_url="https://api.deepseek.com",
        model="deepseek-v4-flash",
        timeout_sec=30.0,
    ):
        if not api_key:
            raise RuntimeError("DeepSeek API key is required.")
        self.api_key = api_key
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.timeout_sec = float(timeout_sec)

    def parse(self, command):
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": command},
            ],
            "response_format": {"type": "json_object"},
            "thinking": {"type": "disabled"},
            "stream": False,
        }
        request = urllib.request.Request(
            self.base_url + "/chat/completions",
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "Authorization": "Bearer " + self.api_key,
            },
            method="POST",
        )

        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                response_data = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError("DeepSeek API HTTP error %s: %s" % (exc.code, body))
        except Exception as exc:
            raise RuntimeError("DeepSeek API request failed: %s" % exc)

        try:
            content = response_data["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError) as exc:
            raise RuntimeError("DeepSeek API returned an unexpected response: %s" % exc)

        try:
            parsed = json.loads(content)
        except json.JSONDecodeError as exc:
            raise RuntimeError(
                "DeepSeek did not return valid JSON: %s; content=%s" % (exc, content)
            )

        return validate_goal_stages(parsed)


def validate_goal_stages(parsed):
    if not isinstance(parsed, dict):
        raise RuntimeError("LLM result must be a JSON object.")

    if parsed.get("ok") is False:
        error = parsed.get("error", "LLM could not parse a supported command.")
        raise RuntimeError(str(error))

    if parsed.get("ok") is not True:
        raise RuntimeError("LLM result field 'ok' must be true or false.")

    stages = parsed.get("stages")
    if stages is None:
        goals = parsed.get("goals")
        if not isinstance(goals, list) or not goals:
            raise RuntimeError(
                "LLM result must include a non-empty goals list or stages list."
            )
        return [{"goals": _normalize_goals(goals, "stage 1")}]

    if not isinstance(stages, list) or not stages:
        raise RuntimeError("LLM result stages must be a non-empty list.")

    normalized_stages = []
    for index, stage in enumerate(stages, start=1):
        if not isinstance(stage, dict):
            raise RuntimeError("Stage %d must be a JSON object." % index)
        goals = stage.get("goals")
        normalized_stages.append(
            {"goals": _normalize_goals(goals, "stage %d" % index)}
        )

    return normalized_stages


def validate_goal_constraints(parsed):
    stages = validate_goal_stages(parsed)
    if len(stages) != 1:
        raise RuntimeError("Expected exactly one stage.")
    return stages[0]["goals"]


def _normalize_goals(goals, label):
    if not isinstance(goals, list) or not goals:
        raise RuntimeError("%s must include a non-empty goals list." % label)

    normalized = []
    legal_targets = set(CELL_COORDS) | set(AREAS)
    for index, goal in enumerate(goals, start=1):
        if not isinstance(goal, dict):
            raise RuntimeError("%s goal %d must be a JSON object." % (label, index))
        cube = str(goal.get("cube", "")).strip().lower()
        if cube not in VALID_CUBES:
            raise RuntimeError(
                "%s goal %d has invalid cube '%s'." % (label, index, cube)
            )

        target_cube = str(goal.get("target_cube_position", "")).strip().lower()
        if target_cube:
            if target_cube not in VALID_CUBES:
                raise RuntimeError(
                    "%s goal %d has invalid target cube '%s'."
                    % (label, index, target_cube)
                )
            normalized.append(
                {"cube": cube, "target_cube_position": target_cube}
            )
            continue

        target = str(goal.get("target", goal.get("place_cell", ""))).strip().lower()
        target = target.replace(" ", "_")
        if target not in legal_targets:
            raise RuntimeError(
                "%s goal %d has invalid target '%s'." % (label, index, target)
            )
        normalized.append({"cube": cube, "target": target})

    return normalized
