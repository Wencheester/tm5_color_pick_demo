import json
import urllib.error
import urllib.request


SYSTEM_PROMPT = """You are a command classifier for a ROS 2 color cube sorting demo.
Return JSON only. Do not use markdown. Do not explain.

The only supported task is color sorting for exactly one red cube, one yellow
cube, and one blue cube.

Classify the user command as color_sort only when the user clearly asks to sort
or classify cubes by color, put cubes into their corresponding color regions,
or uses equivalent Chinese or English phrasing.

Supported examples:
- 把方块按照颜色分类
- 按颜色分类
- 颜色分类
- 把方块放到对应颜色区域
- 按对应颜色放置
- sort by color
- classify the cubes by color

Unsupported examples:
- move yellow to right top
- swap red and blue
- put cubes in the top row

Success schema:
{"ok": true, "task_type": "color_sort"}

Failure schema:
{"ok": false, "error": "short reason"}
"""


class ColorSortGoalParser:
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

        return validate_color_sort_task(parsed)


def validate_color_sort_task(parsed):
    if not isinstance(parsed, dict):
        raise RuntimeError("LLM result must be a JSON object.")

    if parsed.get("ok") is False:
        error = parsed.get("error", "LLM did not classify this as color sorting.")
        raise RuntimeError(str(error))

    if parsed.get("ok") is not True:
        raise RuntimeError("LLM result field 'ok' must be true or false.")

    task_type = str(parsed.get("task_type", "")).strip().lower()
    if task_type != "color_sort":
        raise RuntimeError("Unsupported Demo 6 task_type '%s'." % task_type)

    return {"task_type": "color_sort"}
