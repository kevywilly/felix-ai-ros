"""Tiny OpenAI-compatible chat client for llama.cpp's llama-server.

Stdlib only (urllib) -- no `openai`/`requests` dependency. Talks to the endpoint
stood up by llm_server.sh (default http://localhost:8080/v1). The model is a
reasoning model that emits <think>...</think> traces; strip_think() removes them
from any user-facing text.
"""
import json
import re
import urllib.request

_THINK = re.compile(r"<think>.*?</think>", re.DOTALL)


def strip_think(text):
    if not text:
        return ""
    return _THINK.sub("", text).strip()


def chat(base_url, model, messages, tools=None, timeout=60.0, temperature=0.2):
    """POST /chat/completions and return the assistant `message` dict.

    Raises urllib.error.URLError / TimeoutError on transport failure (the caller
    turns that into a spoken error rather than crashing).
    """
    payload = {
        "model": model,
        "messages": messages,
        "temperature": temperature,
        "stream": False,
    }
    if tools:
        payload["tools"] = tools
        payload["tool_choice"] = "auto"

    req = urllib.request.Request(
        base_url.rstrip("/") + "/chat/completions",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        body = json.loads(resp.read().decode("utf-8"))
    return body["choices"][0]["message"]
