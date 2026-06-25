"""
LLM backend abstraction for the ADRC agent tuner.

Set these env vars to select a provider:

  # Gemini (default)
  LLM_PROVIDER=gemini
  GEMINI_API_KEY=...
  LLM_MODEL=gemini-flash-lite-latest          # optional

  # Any OpenAI-compatible API (Blackbox, OpenAI, local Ollama, etc.)
  LLM_PROVIDER=openai_compat
  LLM_API_KEY=...
  LLM_BASE_URL=https://api.blackbox.ai/api/chat
  LLM_MODEL=blackboxai
  LLM_JSON_MODE=false                         # set false if provider doesn't support response_format
"""

import json
import os
from abc import ABC, abstractmethod
from pydantic import BaseModel


class TuningResult(BaseModel):
    reasoning: str
    phase: str          # "STEP_POS" | "STEP_NEG" | "TUNE" | "VERIFY"
    wc: float
    b0: float
    ramp_time: float
    target_velocity: float


class LLMBackend(ABC):
    @abstractmethod
    def complete(self, system_prompt: str, user_prompt: str) -> TuningResult:
        pass


class GeminiBackend(LLMBackend):
    def __init__(self, api_key: str, model: str = "gemini-flash-lite-latest"):
        from google import genai
        self.client = genai.Client(api_key=api_key)
        self.model = model

    def complete(self, system_prompt: str, user_prompt: str) -> TuningResult:
        from google.genai import types
        response = self.client.models.generate_content(
            model=self.model,
            contents=[
                types.Content(role="user", parts=[
                    types.Part.from_text(text=system_prompt + "\n\n" + user_prompt)
                ])
            ],
            config=types.GenerateContentConfig(
                response_mime_type="application/json",
                response_schema=TuningResult,
                temperature=0.1,
            ),
        )
        return response.parsed


class OpenAICompatibleBackend(LLMBackend):
    """Works with any OpenAI-compatible API: Blackbox, OpenAI, Ollama, etc."""

    def __init__(self, api_key: str, base_url: str, model: str,
                 temperature: float = 0.1, json_mode: bool = True):
        try:
            from openai import OpenAI
            self.client = OpenAI(api_key=api_key or "none", base_url=base_url)
        except ImportError:
            raise ImportError("openai package required: pip install openai")
        self.model = model
        self.temperature = temperature
        self.json_mode = json_mode
        self._schema_hint = json.dumps(TuningResult.model_json_schema(), indent=2)

    def complete(self, system_prompt: str, user_prompt: str) -> TuningResult:
        required_fields = list(TuningResult.model_fields.keys())
        system = (
            system_prompt
            + f"\n\n## Output Format\nRespond with a single JSON object. "
              f"ALL of these fields are REQUIRED: {required_fields}.\n"
              f"Schema:\n{self._schema_hint}\n"
              f"Do not omit any field. Do not add extra text outside the JSON."
        )
        kwargs = dict(
            model=self.model,
            messages=[
                {"role": "system", "content": system},
                {"role": "user", "content": user_prompt},
            ],
            temperature=self.temperature,
        )
        if self.json_mode:
            kwargs["response_format"] = {"type": "json_object"}

        last_err = None
        for attempt in range(3):
            try:
                response = self.client.chat.completions.create(**kwargs)
                content = response.choices[0].message.content.strip()

                # Strip markdown code fences if present
                if content.startswith("```"):
                    content = content.split("```")[1]
                    if content.startswith("json"):
                        content = content[4:]
                    content = content.strip()

                return TuningResult(**json.loads(content))
            except Exception as e:
                last_err = e
                # On retry, add an explicit reminder about missing fields
                kwargs["messages"] = [
                    {"role": "system", "content": system},
                    {"role": "user", "content": user_prompt},
                    {"role": "assistant", "content": content if "content" in dir() else ""},
                    {"role": "user", "content": f"Your response was missing required fields. Error: {e}. Return the COMPLETE JSON with ALL fields: {required_fields}."},
                ]

        raise last_err


def create_backend() -> LLMBackend:
    provider = os.environ.get("LLM_PROVIDER", "gemini").lower()

    if provider == "gemini":
        api_key = os.environ.get("GEMINI_API_KEY", "")
        if not api_key:
            raise ValueError("GEMINI_API_KEY is not set")
        model = os.environ.get("LLM_MODEL", "gemini-flash-lite-latest")
        return GeminiBackend(api_key=api_key, model=model)

    if provider == "openai_compat":
        api_key = os.environ.get("LLM_API_KEY", "")
        base_url = os.environ.get("LLM_BASE_URL", "")
        model = os.environ.get("LLM_MODEL", "")
        if not base_url or not model:
            raise ValueError("LLM_BASE_URL and LLM_MODEL must be set for openai_compat provider")
        json_mode = os.environ.get("LLM_JSON_MODE", "true").lower() != "false"
        return OpenAICompatibleBackend(api_key=api_key, base_url=base_url, model=model, json_mode=json_mode)

    raise ValueError(f"Unknown LLM_PROVIDER='{provider}'. Valid options: gemini, openai_compat")
