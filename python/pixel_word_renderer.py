"""Render words to the mechanical PixelScreen using the 4x3 font and configured UART handshake."""
import argparse
import json
from pathlib import Path
from pixel_screen_client import PixelScreenClient, ProtocolError

FONT_WIDTH = 4
FONT_HEIGHT = 3
FONT_MATRIX = {
    "A": [" ## ", "#  #", "####"],
    "B": ["### ", "#  #", "### "],
    "C": [" ## ", "#   ", " ## "],
    "D": ["### ", "#  #", "### "],
    "E": ["####", "### ", "####"],
    "F": ["####", "### ", "#   "],
    "G": [" ## ", "#  #", " ###"],
    "H": ["#  #", "####", "#  #"],
    "I": ["####", " ## ", "####"],
    "J": ["  ##", "  # ", "##  "],
    "K": ["#  #", "### ", "#  #"],
    "L": ["#   ", "#   ", "####"],
    "M": ["#  #", "####", "#  #"],
    "N": ["#  #", "## #", "#  #"],
    "O": [" ## ", "#  #", " ## "],
    "P": ["### ", "#  #", "#   "],
    "Q": [" ## ", "#  #", " ###"],
    "R": ["### ", "#  #", "## #"],
    "S": [" ## ", "#   ", " ## "],
    "T": ["####", " ## ", " ## "],
    "U": ["#  #", "#  #", " ## "],
    "V": ["#  #", "#  #", " ## "],
    "W": ["#  #", "####", "#  #"],
    "X": ["#  #", " ## ", "#  #"],
    "Y": ["#  #", " ## ", " ## "],
    "Z": ["####", "  # ", "####"],
    "0": [" ## ", "#  #", " ## "],
    "1": ["  # ", " ## ", "  # "],
    "2": [" ## ", " ## ", "##  "],
    "3": ["### ", " ## ", "### "],
    "4": ["#  #", "####", "   #"],
    "5": ["####", "#   ", "### "],
    "6": [" ## ", "#   ", "### "],
    "7": ["####", "  # ", "  # "],
    "8": [" ## ", "### ", "## #"],
    "9": ["### ", "### ", "  # "],
    " ": ["    ", "    ", "    "],
    "?": [" ## ", "  # ", " #  "]
}

FONT_COLUMNS = {}
for key, rows in FONT_MATRIX.items():
    if len(rows) != FONT_HEIGHT or any(len(row) != FONT_WIDTH for row in rows):
        raise ValueError(f"font entry {key!r} must be {FONT_HEIGHT} rows of width {FONT_WIDTH}")
    columns = []
    for x in range(FONT_WIDTH):
        bits = 0
        for y in range(FONT_HEIGHT):
            if rows[y][x] != " ":
                bits |= 1 << y
        columns.append(bits)
    FONT_COLUMNS[key] = columns


class PixelWordRenderer:
    def __init__(self, config_path: Path | str, verbose: bool = False):
        self.config_path = Path(config_path)
        config_data = json.loads(self.config_path.read_text())
        self.serial_config = config_data["serial"]
        setup_section = config_data.get("setup", {})
        self.column_encoding = setup_section.get("column_encoding") or config_data.get("column_encoding")
        if self.column_encoding is None:
            raise ValueError("column_encoding is missing from configuration")
        if len(self.column_encoding) != 16:
            raise ValueError("column_encoding must have 16 entries")
        if not all(0 <= val <= 0x0F for val in self.column_encoding):
            raise ValueError("column_encoding entries must be 0-15")
        font_info = config_data.get("font", {})
        if font_info.get("char_width", FONT_WIDTH) != FONT_WIDTH:
            raise ValueError("This renderer only supports the 4-wide font")
        if font_info.get("char_height", FONT_HEIGHT) != FONT_HEIGHT:
            raise ValueError("This renderer only supports the 3-high font")
        renderer_cfg = config_data.get("renderer", {})
        self.start_column = renderer_cfg.get("start_column", 0)
        self.max_columns = renderer_cfg.get("max_columns", 32)
        self.inter_char_spacing = renderer_cfg.get("inter_char_spacing", 1)
        if not (0 <= self.start_column < self.max_columns <= 32):
            raise ValueError("renderer columns must stay within 0-32")
        serial_port = self.serial_config["port"]
        baudrate = self.serial_config.get("baudrate", 115200)
        timeout = self.serial_config.get("timeout", 1.0)
        crc_enabled = self.serial_config.get("crc_enabled", True)
        hardware_settings = config_data.get("hardware") or {}
        self.client = PixelScreenClient(
            serial_port,
            baudrate,
            timeout,
            verbose=verbose,
            crc_enabled=crc_enabled,
            hardware_settings=hardware_settings,
        )
        self.config_version = setup_section.get("config_version", config_data.get("config_version", 1))
        try:
            self._perform_handshake()
        except Exception:
            self.client.close()
            raise

    def _perform_handshake(self) -> None:
        self.client.configure_column_mapping(self.column_encoding, version=self.config_version)
        status_payload = self.client.status()
        _, detail = status_payload[:2]
        homed = bool(detail & 1)
        remote_version = detail >> 1
        if remote_version != self.config_version:
            raise ProtocolError(
                f"Arduino config version {remote_version} does not match {self.config_version}"
            )
        self._remote_homed = homed

    def close(self) -> None:
        self.client.close()

    def home(self) -> None:
        self.client.home()

    def _word_to_columns(self, text: str) -> list[int]:
        if not text:
            raise ValueError("word must be non-empty")
        columns: list[int] = []
        for char in text.upper():
            columns.extend(FONT_COLUMNS.get(char, FONT_COLUMNS["?"]))
            columns.extend([0] * self.inter_char_spacing)
        if self.inter_char_spacing:
            columns = columns[:-self.inter_char_spacing]
        return columns

    def display_word(self, text: str) -> None:
        pattern = self._word_to_columns(text)
        padded = [0] * self.start_column + pattern
        if len(padded) > self.max_columns:
            raise ValueError(
                f"rendered word requires {len(padded)} columns but max_columns is {self.max_columns}"
            )
        self.client.write_columns(padded)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Render a word on PixelScreen with the 4x3 font.")
    parser.add_argument("word", help="Word to draw on the screen")
    parser.add_argument(
        "--config",
        default="python/pixel_screen_config.json",
        help="Path to the JSON configuration file",
    )
    parser.add_argument("--home", action="store_true", help="Run the homing routine before rendering")
    parser.add_argument("--verbose", action="store_true", help="Enable verbose UART logging")
    args = parser.parse_args()

    renderer = PixelWordRenderer(args.config, verbose=args.verbose)
    try:
        if args.home:
            renderer.home()
        renderer.display_word(args.word)
    finally:
        renderer.close()
