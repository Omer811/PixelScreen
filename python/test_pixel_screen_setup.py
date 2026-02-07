"""Regression tests for the interactive setup prompt helpers."""

from pixel_screen_setup import prompt_direction_confirmation


def test_prompt_direction_confirmation_accepts_default(monkeypatch):
    prompts: list[str] = []

    def fake_input(prompt: str) -> str:
        prompts.append(prompt)
        return ""

    monkeypatch.setattr("builtins.input", fake_input)

    assert prompt_direction_confirmation()
    assert prompts == ["Did the drum move in the expected direction? [Y/n] "]


def test_prompt_direction_confirmation_rejects_no(monkeypatch):
    prompts: list[str] = []

    def fake_input(prompt: str) -> str:
        prompts.append(prompt)
        return "n"

    monkeypatch.setattr("builtins.input", fake_input)

    assert not prompt_direction_confirmation()
    assert prompts == ["Did the drum move in the expected direction? [Y/n] "]
