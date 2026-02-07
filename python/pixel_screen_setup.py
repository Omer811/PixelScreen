"""Interactive calibration/handshake helper for the PixelScreen firmware."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Callable, Optional

from pixel_screen_client import (
    PixelScreenClient,
    ProtocolError,
    CMD_SETUP_START,
    CMD_SETUP_ACK_COLUMN,
    CMD_SETUP_DIRECTION,
    CMD_SETUP_POSITION,
    CMD_RESPONSE,
)

SETUP_DETAIL_COLUMN = 0x80
SETUP_DETAIL_DIRECTION = 0x81
SETUP_DETAIL_STEPS = 0x82
SETUP_DETAIL_POSITION_BASE = 0x90
SETUP_DETAIL_DONE = 0xF0


def run_tuning_sequence(client: PixelScreenClient, columns: int) -> bool:
    """Step each column through bursts so the A4988 pots can be tuned."""
    steps_default = 200
    for column in range(columns):
        print(f"Tuning driver for column {column} (adjust pot while bursts run).")
        while True:
            steps = prompt_int(
                "Burst micro-steps (feel for smooth rotation, 100-500 is typical)", steps_default
            )
            client.tune_column(column, steps)
            action = input("Enter=repeat burst, n=next column, q=abort tuning: ").strip().lower()
            if action == "q":
                return False
            if action == "n":
                break
            steps_default = steps
    return True


def load_config(config_path: Path) -> dict:
    return json.loads(config_path.read_text())


def save_config(config_path: Path, data: dict) -> None:
    config_path.write_text(json.dumps(data, indent=2))


def prompt_bits(position: int) -> str:
    while True:
        value = input(f"Position {position:02}: enter 4 bits (top → bottom, e.g. 1101): ").strip()
        cleaned = ''.join(ch for ch in value if ch in '01')
        if len(cleaned) == 4:
            return cleaned
        print("Please provide exactly four 0/1 digits.")


def prompt_int(prompt: str, default: int) -> int:
    while True:
        raw = input(f"{prompt} [{default}]: ").strip()
        if raw == "":
            return default
        try:
            return int(raw)
        except ValueError:
            print("Please enter a valid number.")


def prompt_direction_confirmation() -> bool:
    """Ask the user whether the drum moved in the expected direction."""
    response = input("Did the drum move in the expected direction? [Y/n] ").strip().lower()
    return response in ("", "y", "yes")


def run_interactive_setup(
    config_path: Path,
    force: bool,
    tune: bool = False,
    tune_count: Optional[int] = None,
    verbose: bool = False,
    client_factory: Optional[Callable[..., PixelScreenClient]] = None,
) -> None:
    config_data = load_config(config_path)
    serial_config = config_data["serial"]
    setup_section = config_data.setdefault("setup", {})
    column_encoding = setup_section.get("column_encoding") or config_data.get("column_encoding")
    config_version = setup_section.get("config_version", config_data.get("config_version", 1))

    hardware_settings = config_data.get("hardware") or {}
    client_creator = client_factory if client_factory is not None else PixelScreenClient
    client = client_creator(
        serial_config["port"],
        serial_config.get("baudrate", 115200),
        serial_config.get("timeout", 1.0),
        verbose=verbose,
        crc_enabled=serial_config.get("crc_enabled", True),
        hardware_settings=hardware_settings,
    )

    try:
        total_motors = setup_section.get(
            "motor_count", config_data.get("renderer", {}).get("max_columns", 32)
        )
        tune_columns_default = min(
            setup_section.get("tune_columns", min(total_motors, 4)), total_motors
        )
        tune_columns = tune_count if tune_count and tune_count > 0 else tune_columns_default
        if tune and tune_columns > 0:
            print(
                f"Tuning current for the first {tune_columns} of {total_motors} columns."
            )
            client.home()
            if not run_tuning_sequence(client, tune_columns):
                print("Tuning aborted; stop setup.")
                return
            setup_section["tune_columns"] = tune_columns
        setup_section["motor_count"] = total_motors
        setup_section.setdefault("tune_columns", tune_columns_default)
        if column_encoding and setup_section.get("completed") and not force:
            use = input("Existing calibration found. Send it to the Arduino? [Y/n] ").strip().lower()
            if use in ("", "y", "yes"):
                client.configure_column_mapping(
                    column_encoding, version=config_version, motor_count=total_motors
                )
                client.home()
                print("Existing mapping uploaded and the gantry was homed.")
                return
        print("Starting interactive setup - this will walk the Arduino through homing/encoding.")
        client.start_interactive_setup()

        position_bits: list[str] = []
        steps = 0
        direction_forward = True
        while True:
            cmd, payload = client.read_response()
            if cmd != CMD_RESPONSE or len(payload) < 2:
                raise ProtocolError("unexpected interactive response")
            status, detail = payload[0], payload[1]
            extra = payload[2:]
            if status != 0:
                raise ProtocolError(f"interactive error {status}")

            if detail >= SETUP_DETAIL_POSITION_BASE and detail < SETUP_DETAIL_POSITION_BASE + 16:
                pos = detail - SETUP_DETAIL_POSITION_BASE
                bits = prompt_bits(pos)
                position_bits.append(bits)
                client.send_setup_command(CMD_SETUP_POSITION, bits.encode("ascii"))
            elif detail == SETUP_DETAIL_DIRECTION:
                direction_forward = prompt_direction_confirmation()
                client.send_setup_command(CMD_SETUP_DIRECTION, b"\x01" if direction_forward else b"\x00")
            elif detail == SETUP_DETAIL_STEPS:
                if len(extra) >= 2:
                    steps = extra[0] | (extra[1] << 8)
                    print(f"Measured {steps} steps per revolution.")
                else:
                    print("Arduino reported steps but payload was missing.")
            elif detail >= SETUP_DETAIL_COLUMN and detail < SETUP_DETAIL_DIRECTION:
                column = detail & 0x1F
                print(f"Column {column} homed. Sending acknowledgment...")
                client.send_setup_command(CMD_SETUP_ACK_COLUMN, bytes((column,)))
            elif detail == SETUP_DETAIL_DONE:
                print("Interactive setup complete.")
                break
            else:
                print(f"Ignoring unexpected detail {detail:02X}")

        if len(position_bits) != 16:
            raise ProtocolError("did not collect 16 position patterns")

        values = [int(bits, 2) for bits in position_bits]
        if len(set(values)) != 16:
            raise ProtocolError("position patterns were not unique")

        mapping = [0x00] * 16
        for pos, value in enumerate(values):
            mapping[value] = pos

        old_version = setup_section.get("config_version", config_data.get("config_version", 1))
        new_version = old_version + 1
        setup_section.update(
            {
                "config_version": new_version,
                "column_encoding": mapping,
                "position_patterns": position_bits,
                "steps_per_revolution": steps,
                "direction_forward": direction_forward,
                "completed": True,
            }
        )
        client.configure_column_mapping(mapping, version=new_version, motor_count=total_motors)
        client.home()
        save_config(config_path, config_data)
        print("Calibration data saved to configuration file.")
    finally:
        try:
            client.abort_setup()
        except ProtocolError:
            pass
        client.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run or replay the PixelScreen interactive calibration.")
    parser.add_argument(
        "--config",
        default="python/pixel_screen_config.json",
        help="Path to the JSON configuration file",
    )
    parser.add_argument("--force", action="store_true", help="Skip reusing stored config and always re-run calibration")
    parser.add_argument("--verbose", action="store_true", help="Enable verbose UART logging")
    parser.add_argument("--tune", action="store_true", help="Run the integrated current-limit tuning before calibration")
    parser.add_argument("--tune-count", type=int, default=0, help="How many columns to run tuning on (overrides config)")
    args = parser.parse_args()

    run_interactive_setup(
        Path(args.config),
        force=args.force,
        tune=args.tune,
        tune_count=args.tune_count,
        verbose=args.verbose,
    )
