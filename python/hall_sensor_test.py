"""Poll and display the hall-effect sensor state via the PixelScreen protocol."""
import argparse
import json
import time
from pathlib import Path

from pixel_screen_client import PixelScreenClient


def load_config(config_path: Path) -> dict:
    return json.loads(config_path.read_text())


def display_states(bits: list[int]) -> None:
    for row in range(4):
        cells = []
        for col in range(8):
            index = row * 8 + col
            byte_index = index // 8
            bit_pos = index % 8
            active = (bits[byte_index] >> bit_pos) & 1
            cells.append("1" if active else ".")
        print(f"Columns {row*8:02}-{row*8+7:02}: {' '.join(cells)}")


def run_test(config_path: Path, interval: float, verbose: bool = False) -> None:
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
        client.abort_setup()
        print("Polling MТP23017 hall sensors. Move a magnet near each sensor to see it go LOW.")
        while True:
            bits = client.read_hall_sensors()
            display_states(bits)
            time.sleep(interval)
    finally:
        client.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Poll the PixelScreen hall-effect sensors.")
    parser.add_argument(
        "--config",
        default="python/pixel_screen_config.json",
        help="Path to the JSON configuration file",
    )
    parser.add_argument("--interval", type=float, default=0.5, help="Seconds between polls")
    parser.add_argument("--verbose", action="store_true", help="Enable verbose UART logging")
    args = parser.parse_args()

    run_test(Path(args.config), args.interval, verbose=args.verbose)
