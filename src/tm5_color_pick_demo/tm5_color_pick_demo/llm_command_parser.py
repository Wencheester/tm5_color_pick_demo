import json
import urllib.error
import urllib.request


VALID_CUBES = {"red", "yellow", "blue"}
VALID_CELLS = {
    "left_top",
    "left_middle",
    "left_bottom",
    "center_top",
    "center_middle",
    "center_bottom",
    "right_top",
    "right_middle",
    "right_bottom",
}


SYSTEM_PROMPT = """You are a robot command parser for a ROS 2 pick-and-place demo.
Return JSON only. Do not use markdown. Do not explain.

The robot can only execute ordered single-cube move tasks.
Legal cube values: red, yellow, blue.
Legal place_cell values: left_top, left_middle, left_bottom, center_top, center_middle, center_bottom, right_top, right_middle, right_bottom.

Map colors from any language when clear:
red / 红 / 红色 / red cube -> red
yellow / 黄 / 黄色 / yellow cube -> yellow
blue / 蓝 / 蓝色 / blue cube -> blue

Map target cells from natural language when clear:
left top / top left / 左上 / 上左 -> left_top
left middle / middle left / 左中 / 中左 -> left_middle
left bottom / bottom left / 左下 / 下左 -> left_bottom
center top / top center / 中上 / 上中 -> center_top
center / middle / center middle / 中间 / 中心 / 正中 -> center_middle
center bottom / bottom center / 中下 / 下中 -> center_bottom
right top / top right / 右上 / 上右 -> right_top
right middle / middle right / 右中 / 中右 -> right_middle
right bottom / bottom right / 右下 / 下右 -> right_bottom

If the user provides multiple tasks, output them in order.
Resolve pronouns such as it / that cube / 它 / 这个 / 那个 to the most recent explicit cube.
If a task is missing cube or target, or asks for unsupported planning, output ok=false.

Success schema:
{"ok": true, "tasks": [{"cube": "red", "place_cell": "right_top"}]}
Failure schema:
{"ok": false, "error": "short reason"}
"""


class LlmCommandParser:
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
            raise RuntimeError("DeepSeek did not return valid JSON: %s; content=%s" % (exc, content))

        return validate_llm_tasks(parsed)


def validate_llm_tasks(parsed):
    if not isinstance(parsed, dict):
        raise RuntimeError("LLM result must be a JSON object.")

    if parsed.get("ok") is False:
        error = parsed.get("error", "LLM could not parse a supported command.")
        raise RuntimeError(str(error))

    if parsed.get("ok") is not True:
        raise RuntimeError("LLM result field 'ok' must be true or false.")

    tasks = parsed.get("tasks")
    if not isinstance(tasks, list) or not tasks:
        raise RuntimeError("LLM result must include a non-empty tasks list.")

    normalized = []
    for index, task in enumerate(tasks, start=1):
        if not isinstance(task, dict):
            raise RuntimeError("Task %d must be a JSON object." % index)
        cube = str(task.get("cube", "")).strip().lower()
        place_cell = str(task.get("place_cell", "")).strip()
        if cube not in VALID_CUBES:
            raise RuntimeError("Task %d has invalid cube '%s'." % (index, cube))
        if place_cell not in VALID_CELLS:
            raise RuntimeError(
                "Task %d has invalid place_cell '%s'." % (index, place_cell)
            )
        normalized.append({"cube": cube, "place_cell": place_cell})

    return normalized
