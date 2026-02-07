"""Assist in tuning the A4988 current limit via the PixelScreen interface.

Set `setup.motor_count` in the JSON config to reflect how many drivers you eventually wire;
then you can tune any column index within that range. This helper homes the system and sends bursts
on the requested column so you can adjust the pot before continuing.
"""
import argparse
import json
from pathlib import Path

from pixel_screen_client import PixelScreenClient


def load_config(config_path: Path) -> dict:
    return json.loads(config_path.read_text())


def prompt_int(prompt: str, default: int) -> int:
    while True:
        raw = input(f"{prompt} [{default}]: ").strip()
        if raw == "":
            return default
        try:
            return int(raw)
        except ValueError:
            print("Please enter a number.")


def run_tuning(config_path: Path, column: int, verbose: bool = False) -> None:
    config = load_config(config_path)
    serial_config = config["serial"]

    hardware_settings = config.get("hardware") or {}
    client = PixelScreenClient(
        serial_config["port"],
        serial_config.get("baudrate", 115200),
        serial_config.get("timeout", 1.0),
        verbose=verbose,
        crc_enabled=serial_config.get("crc_enabled", True),
        hardware_settings=hardware_settings,
    )

    try:
        print("Homing all steppers before tuning.")
        client.home()
        print(f"Starting tuning on column {column}.")
        steps_default = 200
        while True:
            steps = prompt_int("Enter number of micro-steps to apply (per burst)", steps_default)
            client.tune_column(column, steps)
            print("Motor stepped. Listen/feel for skipping or excessive heat.")
            answer = input("Dial set? [enter to repeat, c to change column, q to quit]: ").strip().lower()
            if answer == "q":
                break
            if answer == "c":
                column = prompt_int("Column index (0-31)", column)
            steps_default = steps
    finally:
        client.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tuning helper for the A4988 current limit via PixelScreen.")
    parser.add_argument(
        "--config",
        default="python/pixel_screen_config.json",
        help="Path to the JSON configuration file",
    )
    parser.add_argument("--column", type=int, default=0, help="Column number whose driver you want to tune")
    parser.add_argument("--verbose", action="store_true", help="Enable verbose UART logging")
    args = parser.parse_args()

    run_tuning(Path(args.config), args.column, verbose=args.verbose)
