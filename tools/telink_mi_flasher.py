#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = [
#   "bleak>=0.21.0",
# ]
# ///
"""
Linux CLI port of pvvx's TelinkMiFlasher.html, focused on Xiaomi LYWSD03MMC
firmware 2.1.1_0159 OTA flashing to custom pvvx/ATC firmware.

Ported directly from:
  pvvx.github.io/ATC_MiThermometer/TelinkMiFlasher.html (v14.3)

Key capabilities for firmware 2.1.1_0159:
  - Mi Home Token authentication over BLE service 0xfe95 (both MiBLE v2
    extended-MTU frames used by 2.1.1_0159 and classic MiBLE v1 chunked frames)
    implemented in pure Python stdlib (HKDF-SHA256 + HMAC-SHA256).
  - Automatic firmware validation (Telink 'TLNK' header + un-inverted Telink
    CRC32 + Zigbee 0x0BEEF11E wrapper extraction + MITFOTA detection).
  - Automatic 2.1.1_0159c exploit signature generation & patching from
    `sign_2.1.1_0159c.bin` (`getSignFile` port).
  - Support for 2025+ hardware revisions (B1.5 / B1.6 / B1.1) requiring the
    two-stage update via `Original_OTA_Xiaomi_LYWSD03MMC_v2.1.1_0159c.bin`
    followed by `ATC_vNN.bin` (`--signed-original` / `--two-stage`, see
    https://github.com/pvvx/ATC_MiThermometer/issues/602).
  - Direct parsing of `Xiaomi-cloud-tokens-extractor` JSON (`-o tokens.json`)
    or saved terminal output (`--tokens-file`) for single or batch flashing.

Runtime BLE dependency on Linux:
  pip install bleak
"""

from __future__ import annotations

import argparse
import asyncio
import dataclasses
import hashlib
import hmac
import json
import os
import re
import secrets
import struct
import sys
import time
import urllib.error
import urllib.request
import zlib
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple


# ---------------------------------------------------------------------------
# Constants & GATT UUIDs (matching TelinkMiFlasher.html)
# ---------------------------------------------------------------------------

MAX_BLE_OTA_SIZE = 0x20000  # 128 KiB
MAX_EXT_OTA_SIZE = 0x34000  # 208 KiB

TELINK_MAGIC = 0x544C4E4B  # "KNLT" LE -> "TLNK" at offset 0x08
ZIGBEE_OTA_MAGIC = 0x0BEEF11E
MITFOTA_HEADER = bytes.fromhex("000000004d49ef54464f5441")  # \x00*4 + "MI\xefTFOTA"

SIGN_FILE_211_0159 = "sign_2.1.1_0159c.bin"
RAW_REPO_BASE_URL = "https://raw.githubusercontent.com/pvvx/ATC_MiThermometer/master/"

UUID_SVC_TELINK_OTA = "00010203-0405-0607-0809-0a0b0c0d1912"
UUID_CHR_TELINK_OTA = "00010203-0405-0607-0809-0a0b0c0d2b12"

UUID_SVC_MI_MAIN = "ebe0ccb0-7a0a-4b0c-8a1a-6ff2997da3a6"
UUID_CHR_MI_SPEED = "ebe0ccd8-7a0a-4b0c-8a1a-6ff2997da3a6"
UUID_CHR_MI_TEMP = "ebe0ccc1-7a0a-4b0c-8a1a-6ff2997da3a6"

UUID_SVC_MI_AUTH = "0000fe95-0000-1000-8000-00805f9b34fb"
UUID_CHR_MI_ENC_10 = "00000010-0000-1000-8000-00805f9b34fb"
UUID_CHR_MI_ENC_19 = "00000019-0000-1000-8000-00805f9b34fb"

UUID_SVC_GAP = "00001800-0000-1000-8000-00805f9b34fb"
UUID_CHR_GAP_NAME = "00002a00-0000-1000-8000-00805f9b34fb"

UUID_SVC_DEV_INFO = "0000180a-0000-1000-8000-00805f9b34fb"
UUID_CHR_FW_REV = "00002a26-0000-1000-8000-00805f9b34fb"
UUID_CHR_HW_REV = "00002a27-0000-1000-8000-00805f9b34fb"
UUID_CHR_SW_REV = "00002a28-0000-1000-8000-00805f9b34fb"

UUID_SVC_CUSTOM = "00001f10-0000-1000-8000-00805f9b34fb"
UUID_CHR_CUSTOM_CFG = "00001f1f-0000-1000-8000-00805f9b34fb"
UUID_SVC_ENV_SENS = "0000181a-0000-1000-8000-00805f9b34fb"
UUID_SVC_BTHOME = "0000fcd2-0000-1000-8000-00805f9b34fb"

OTA_ERROR_TABLE = (
    "Success",
    "Lost one or more packets",
    "CRC error in data",
    "Writing data to flash",
    "Lost last one or more packets",
    "Timeout",
    "Firmware CRC check",
)

ENC_10_STATUS_MESSAGES = {
    "11000000": ("REG_SUCCESS", True),
    "12000000": ("Registration failed (REG_FAILED)", False),
    "13000000": ("Registration verify successful (REG_VERIFY_SUCC)", True),
    "14000000": ("Registration verify failed (REG_VERIFY_FAIL)", False),
    "21000000": ("Login successful (LOG_SUCCESS)", True),
    "22000000": ("Login invalid LTMK — Mi Token does not match this device!", False),
    "23000000": ("Login failed (LOG_FAILED) — check Mi Token!", False),
    "e0000000": (
        "Device is NOT registered in Mi Home (ERR_NOT_REGISTERED)! "
        "Firmware 2.1.1_0159 must be added in the Mi Home app first.",
        False,
    ),
    "e1000000": ("Error: device already registered (ERR_REGISTERED)", False),
    "e2000000": ("Repeat login accepted (ERR_REPEAT_LOGIN)", True),
    "e3000000": ("Error: invalid OOB (ERR_INVALID_OOB)", False),
}

LYWSD03MMC_HW_MAP: Dict[str, Tuple[int, int]] = {
    # prefix -> (cfg.hver, hwver_id)
    "B1.4": (0x80, 0),
    "0000": (0x80, 0),
    "B1.5": (10, 10),
    "B1.6": (3, 3),
    "B1.9": (4, 4),
    "B1.7": (5, 5),
    "B2.0": (5, 5),
    "B1.1": (14, 14),  # B1.6 / B1.1 (2025)
}

ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
HEX_RE = re.compile(r"^[0-9a-fA-F]+$")


class FlasherError(RuntimeError):
    """Raised on protocol, validation, authentication, or OTA errors."""


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class MiCredentials:
    mac: Optional[str] = None
    name: Optional[str] = None
    did: Optional[str] = None
    token: str = ""
    bind_key: Optional[str] = None
    model: Optional[str] = None


@dataclasses.dataclass
class FirmwareInfo:
    payload: bytes
    header_size: int
    file_crc: int
    already_signed: bool
    is_zigbee_ota: bool = False
    zigbee_man_code: Optional[int] = None
    zigbee_image_type: Optional[int] = None
    zigbee_fw_version: Optional[int] = None


@dataclasses.dataclass
class PvvxConfig:
    """
    Decoded representation of pvvx/ATC custom firmware `0x55` config block
    (matching `CustomBlkParse` and `SendCustomConfig` in TelinkMiFlasher.html).
    """

    ver: int
    flg: int
    flg2: int
    temp_offset: int  # signed int8 (-128..127); for ver > 0x50 low nibble = adv_delay
    humi_offset: int  # signed int8 (-128..127); for ver > 0x50 unsigned = event_adv_cnt
    advertising_interval: int  # 1..255 in 62.5 ms steps
    measure_interval: int  # 1..255 multiplier of advertising_interval
    rf_tx_power: int  # 23..63 (VBAT) or 130..191 (VANT)
    connect_latency: int  # 0..49 (ver > 0x40) or 0..200, in (x+1)*20 ms
    lcd_tint: int = 55  # 10..255 in 0.05 s steps
    hver: int = 0x80
    av_meas_mem: int = 0  # 0..255

    @classmethod
    def from_bytes(cls, pkt: bytes) -> "PvvxConfig":
        if len(pkt) < 10 or pkt[0] != 0x55:
            raise ValueError(
                f"Invalid pvvx 0x55 config packet (len={len(pkt)}, header=0x{pkt[0]:02x})."
            )
        ver = pkt[1]
        flg = pkt[2]
        flg2 = pkt[3]
        temp_offset, humi_offset = struct.unpack_from("<bb", pkt, 4)
        adv_int = pkt[6]
        meas_int = pkt[7]
        rf_tx = pkt[8]
        con_lat = pkt[9]
        lcd_tint = pkt[10] if len(pkt) >= 11 else 55
        if len(pkt) >= 12:
            hver = pkt[11]
            if ver < 0x36:
                hver &= 0x87
        else:
            hver = 0x80
        av_meas_mem = pkt[12] if len(pkt) >= 13 else 0
        return cls(
            ver=ver,
            flg=flg,
            flg2=flg2,
            temp_offset=temp_offset,
            humi_offset=humi_offset,
            advertising_interval=adv_int,
            measure_interval=meas_int,
            rf_tx_power=rf_tx,
            connect_latency=con_lat,
            lcd_tint=lcd_tint,
            hver=hver,
            av_meas_mem=av_meas_mem,
        )

    def to_set_packet(self) -> bytes:
        base = struct.pack(
            "<BBBbbBBBB",
            0x55,
            self.flg & 0xFF,
            self.flg2 & 0xFF,
            max(-128, min(127, int(self.temp_offset))),
            max(-128, min(127, int(self.humi_offset))),
            max(1, min(255, int(self.advertising_interval))),
            max(1, min(255, int(self.measure_interval))),
            self.rf_tx_power & 0xFF,
            self.connect_latency & 0xFF,
        )
        lcd = max(10, min(255, int(self.lcd_tint))) & 0xFF
        if self.ver < 0x20:
            return base + bytes([lcd])
        return base + bytes([lcd, self.hver & 0xFF, max(0, min(255, int(self.av_meas_mem))) & 0xFF])

    @property
    def ver_str(self) -> str:
        return f"{self.ver >> 4}.{self.ver & 0x0F}"

    @property
    def hwver_id(self) -> int:
        if self.ver < 0x48:
            hid = self.hver & 0x0F
            if hid == 0x0F:
                return 16 + (self.lcd_tint & 0x7F)
            return hid
        return self.hver

    @property
    def has_lcd_ui(self) -> bool:
        hid = self.hwver_id
        return hid <= 15 or (30 <= hid <= 33) or hid in (38, 49, 52, 62)

    @property
    def is_clock_only_hw(self) -> bool:
        return self.hwver_id in (9, 12, 49, 62)

    def _adv_type_map(self) -> Dict[int, str]:
        if not self.has_lcd_ui and self.ver < 0x48:
            return {0: "bthome2", 1: "pvvx", 2: "mijia", 3: "bthome1"}
        if self.ver >= 0x45:
            last = "bthome2"
        elif self.ver >= 0x37:
            last = "bthome1"
        else:
            last = "all"
        return {0: "atc1441", 1: "pvvx", 2: "mijia", 3: last}

    @property
    def adv_type(self) -> str:
        return self._adv_type_map().get(self.flg & 0x03, str(self.flg & 0x03))

    def set_adv_type(self, value: str) -> None:
        v = value.strip().lower()
        aliases = {
            "atc": "atc1441",
            "atc1441": "atc1441",
            "pvvx": "pvvx",
            "custom": "pvvx",
            "mijia": "mijia",
            "mi": "mijia",
            "mihome": "mijia",
            "bthome": "bthome2",
            "bthome2": "bthome2",
            "bthomev2": "bthome2",
            "bthome1": "bthome1",
            "bthomev1": "bthome1",
            "all": "all",
        }
        if v in ("0", "1", "2", "3"):
            code = int(v)
        else:
            canon = aliases.get(v)
            if canon is None:
                raise ValueError(f"Unknown advertising type '{value}'.")
            rev_map = {name: k for k, name in self._adv_type_map().items()}
            if canon not in rev_map:
                raise ValueError(
                    f"Advertising type '{value}' is not supported by custom firmware v{self.ver_str} "
                    f"(supported: {', '.join(rev_map.keys())})."
                )
            code = rev_map[canon]
        self.flg = (self.flg & ~0x03) | (code & 0x03)

    @property
    def comfort(self) -> bool:
        return bool(self.flg & 0x04)

    @comfort.setter
    def comfort(self, enabled: bool) -> None:
        self.flg = (self.flg | 0x04) if enabled else (self.flg & ~0x04)

    @property
    def blinking(self) -> bool:
        return bool(self.flg & 0x08)

    @blinking.setter
    def blinking(self, enabled: bool) -> None:
        self.flg = (self.flg | 0x08) if enabled else (self.flg & ~0x08)

    @property
    def temp_unit(self) -> str:
        return "F" if (self.flg & 0x10) else "C"

    @temp_unit.setter
    def temp_unit(self, unit: str) -> None:
        u = unit.strip().upper()
        if u not in ("C", "F"):
            raise ValueError(f"Invalid temperature unit '{unit}' (expected 'C' or 'F').")
        self.flg = (self.flg | 0x10) if u == "F" else (self.flg & ~0x10)

    @property
    def show_batt(self) -> bool:
        mask = 0x08 if self.hwver_id == 49 else 0x20
        return bool(self.flg & mask)

    @show_batt.setter
    def show_batt(self, enabled: bool) -> None:
        mask = 0x08 if self.hwver_id == 49 else 0x20
        self.flg = (self.flg | mask) if enabled else (self.flg & ~mask)

    @property
    def tx_measures(self) -> bool:
        return bool(self.flg & 0x40)

    @tx_measures.setter
    def tx_measures(self, enabled: bool) -> None:
        self.flg = (self.flg | 0x40) if enabled else (self.flg & ~0x40)

    @property
    def lp_measures(self) -> bool:
        return bool(self.flg & 0x80)

    @lp_measures.setter
    def lp_measures(self, enabled: bool) -> None:
        self.flg = (self.flg | 0x80) if enabled else (self.flg & ~0x80)

    @property
    def smiley(self) -> int:
        return self.flg2 & 0x07

    @smiley.setter
    def smiley(self, val: int) -> None:
        if not 0 <= val <= 7:
            raise ValueError("Smiley / screen type must be in range 0..7.")
        self.flg2 = (self.flg2 & ~0x07) | (val & 0x07)

    @property
    def encrypted(self) -> bool:
        return bool(self.flg2 & 0x08)

    @encrypted.setter
    def encrypted(self, enabled: bool) -> None:
        self.flg2 = (self.flg2 | 0x08) if enabled else (self.flg2 & ~0x08)

    @property
    def adv_flags(self) -> bool:
        return bool(self.flg2 & 0x10)

    @adv_flags.setter
    def adv_flags(self, enabled: bool) -> None:
        self.flg2 = (self.flg2 | 0x10) if enabled else (self.flg2 & ~0x10)

    @property
    def bt5_phy(self) -> bool:
        return bool(self.flg2 & 0x20)

    @bt5_phy.setter
    def bt5_phy(self, enabled: bool) -> None:
        self.flg2 = (self.flg2 | 0x20) if enabled else (self.flg2 & ~0x20)

    @property
    def long_range(self) -> bool:
        return bool(self.flg2 & 0x40)

    @long_range.setter
    def long_range(self, enabled: bool) -> None:
        self.flg2 = (self.flg2 | 0x40) if enabled else (self.flg2 & ~0x40)

    @property
    def screen_off(self) -> bool:
        return bool(self.flg2 & 0x80)

    @screen_off.setter
    def screen_off(self, enabled: bool) -> None:
        self.flg2 = (self.flg2 | 0x80) if enabled else (self.flg2 & ~0x80)

    @property
    def adv_interval_ms(self) -> float:
        return self.advertising_interval * 62.5

    def set_adv_interval_ms(self, ms: float) -> None:
        steps = int(round(ms / 62.5))
        self.advertising_interval = max(1, min(255, steps))

    @property
    def measure_interval_s(self) -> float:
        return (self.adv_interval_ms * self.measure_interval) / 1000.0

    def set_measure_interval(self, mult: int) -> None:
        self.measure_interval = max(1, min(255, int(mult)))

    @property
    def connect_latency_ms(self) -> int:
        return (self.connect_latency + 1) * 20

    def set_connect_latency_ms(self, ms: float) -> None:
        lat = int(round((ms / 20.0) - 1.0))
        max_lat = 49 if self.ver > 0x40 else 200
        self.connect_latency = max(0, min(max_lat, lat))

    @property
    def lcd_refresh_s(self) -> float:
        return round(self.lcd_tint * 0.05, 2)

    def set_lcd_refresh_s(self, sec: float) -> None:
        val = int(round(sec / 0.05))
        self.lcd_tint = max(10, min(255, val))

    def set_rf_tx_power(self, val: int) -> None:
        v = int(val)
        if v & 0x80:
            v = max(130, min(191, v))
        else:
            v = max(23, min(63, v))
        self.rf_tx_power = v

    @property
    def temp_offset_c(self) -> float:
        return self.temp_offset / 10.0

    def set_temp_offset_c(self, offset_c: float) -> None:
        if self.ver >= 0x47:
            raise ValueError(
                f"Simple temperature offset in 0x55 is only used on firmware < 4.7 "
                f"(current v{self.ver_str}); use --sensor-cal on >= 4.7."
            )
        self.temp_offset = max(-127, min(127, int(round(offset_c * 10.0))))

    @property
    def humi_offset_pct(self) -> float:
        return self.humi_offset / 10.0

    def set_humi_offset_pct(self, offset_pct: float) -> None:
        if self.ver >= 0x47:
            raise ValueError(
                f"Simple humidity offset in 0x55 is only used on firmware < 4.7 "
                f"(current v{self.ver_str}); use --sensor-cal on >= 4.7."
            )
        self.humi_offset = max(-127, min(127, int(round(offset_pct * 10.0))))

    @property
    def adv_delay_steps(self) -> int:
        return self.temp_offset & 0x0F

    def set_adv_delay_steps(self, steps: int) -> None:
        s = max(0, min(15, int(steps)))
        self.temp_offset = (self.temp_offset & 0xF0) | s

    @property
    def event_adv_cnt(self) -> int:
        return self.humi_offset & 0xFF

    def set_event_adv_cnt(self, count: int) -> None:
        c = max(5, min(255, int(count)))
        if c >= 128:
            c -= 256
        self.humi_offset = c

    def to_dict(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {
            "version": self.ver_str,
            "hwver_id": self.hwver_id,
            "adv_type": self.adv_type,
            "adv_flags": self.adv_flags,
            "encrypted": self.encrypted,
            "bt5_phy": self.bt5_phy,
            "long_range": self.long_range,
            "screen_off": self.screen_off,
            "temp_unit": self.temp_unit,
            "comfort": self.comfort,
            "blinking": self.blinking,
            "show_batt": self.show_batt,
            "smiley": self.smiley,
            "tx_measures": self.tx_measures,
            "lp_measures": self.lp_measures,
            "adv_interval_ms": self.adv_interval_ms,
            "measure_interval": self.measure_interval,
            "measure_interval_s": round(self.measure_interval_s, 2),
            "rf_tx_power": self.rf_tx_power,
            "connect_latency_ms": self.connect_latency_ms,
            "lcd_refresh_s": self.lcd_refresh_s,
            "av_meas_mem": self.av_meas_mem,
            "raw_hex": self.to_set_packet().hex(),
        }
        if self.ver < 0x47:
            d["temp_offset_c"] = self.temp_offset_c
            d["humi_offset_pct"] = self.humi_offset_pct
        elif self.ver > 0x50 and not self.is_clock_only_hw:
            d["adv_delay_ms"] = round(self.adv_delay_steps * 0.625, 3)
            d["event_adv_cnt"] = self.event_adv_cnt
        return d

    def format_summary(self) -> str:
        lines = [
            f"  Adv Type:        {self.adv_type} (encrypted={'yes' if self.encrypted else 'no'}, "
            f"ad_flags={'on' if self.adv_flags else 'off'})",
            f"  BLE PHY:         BT5+={'on' if self.bt5_phy else 'off'}, "
            f"LongRange={'on' if self.long_range else 'off'}, RF_TX={self.rf_tx_power}",
            f"  Intervals:       adv={self.adv_interval_ms:.1f} ms, "
            f"measure={self.measure_interval}x ({self.measure_interval_s:.1f} s), "
            f"conn_latency={self.connect_latency_ms} ms",
            f"  Display:         unit={self.temp_unit}, screen_off={'yes' if self.screen_off else 'no'}, "
            f"comfort={'on' if self.comfort else 'off'}, batt={'on' if self.show_batt else 'off'}, "
            f"smiley={self.smiley}, lcd_tint={self.lcd_refresh_s:.2f} s",
            f"  Measurements:    tx_measures={'on' if self.tx_measures else 'off'}, "
            f"lp_measures={'on' if self.lp_measures else 'off'}, mem_avg={self.av_meas_mem}",
        ]
        if self.ver < 0x47:
            lines.append(
                f"  Offsets:         temp={self.temp_offset_c:+.1f} °C, humi={self.humi_offset_pct:+.1f} %"
            )
        elif self.ver > 0x50 and not self.is_clock_only_hw:
            lines.append(
                f"  Adv Extra:       adv_delay={self.adv_delay_steps} ({self.adv_delay_steps * 0.625:.3f} ms), "
                f"event_adv_cnt={self.event_adv_cnt}"
            )
        return "\n".join(lines)


@dataclasses.dataclass
class DeviceProfile:
    address: str
    name: str
    fw_rev: Optional[str] = None
    hw_rev: Optional[str] = None
    sw_rev: Optional[str] = None
    ota_enabled: bool = False
    mi_enabled: bool = False
    mi_auth_enabled: bool = False
    custom_enabled: bool = False
    bthome_enabled: bool = False
    big_ota_enabled: bool = False
    hwver_id: Optional[int] = None
    custom_ver: Optional[int] = None
    temperature_c: Optional[float] = None
    humidity_pct: Optional[int] = None
    battery_mv: Optional[int] = None
    pvvx_config: Optional[PvvxConfig] = None

    @property
    def is_lywsd03mmc(self) -> bool:
        return (
            (self.name or "").startswith("LYWSD03MMC")
            or (self.fw_rev or "").startswith("2.1.1_0159")
        )

    @property
    def is_fw_211_0159(self) -> bool:
        return (self.fw_rev or "").startswith("2.1.1_0159")

    @property
    def required_sign_file(self) -> Optional[str]:
        if self.is_fw_211_0159:
            return SIGN_FILE_211_0159
        return None

    @property
    def is_2025_hw_variant(self) -> bool:
        hw = (self.hw_rev or "")[:4]
        return hw in ("B1.5", "B1.6", "B1.1")


# ---------------------------------------------------------------------------
# Pure Cryptographic, Checksum & Framing Helpers
# ---------------------------------------------------------------------------

def normalize_mac(mac: str) -> str:
    """Normalize a BLE MAC address to uppercase colon-separated AA:BB:CC:DD:EE:FF."""
    cleaned = re.sub(r"[^0-9a-fA-F]", "", mac or "")
    if len(cleaned) != 12:
        raise ValueError(f"Invalid BLE MAC address '{mac}' (expected 12 hex digits).")
    return ":".join(cleaned[i : i + 2].upper() for i in range(0, 12, 2))


def validate_mi_credentials(token: str, bind_key: Optional[str] = None) -> Tuple[str, Optional[str]]:
    """Validate Mi Token (24 hex chars = 12 bytes) and optional Mi Bind Key (32 hex chars = 16 bytes)."""
    tk = (token or "").strip().lower()
    bk = (bind_key or "").strip().lower() or None

    if len(tk) == 32 and (bk is None or len(bk) == 24):
        raise ValueError(
            "It looks like you passed the 32-hex Mi Bind Key (BLE KEY) as --token! "
            "Mi Token must be 24 hex characters (12 bytes), and Bind Key is 32 hex characters (16 bytes)."
        )
    if len(tk) != 24 or not HEX_RE.match(tk):
        raise ValueError(
            f"Mi Token must be exactly 24 hex characters (12 bytes), got {len(tk)} chars."
        )
    if bk is not None and (len(bk) != 32 or not HEX_RE.match(bk)):
        raise ValueError(
            f"Mi Bind Key must be exactly 32 hex characters (16 bytes), got {len(bk)} chars."
        )
    return tk, bk


def hkdf_sha256(ikm: bytes, length: int, salt: bytes = b"", info: bytes = b"") -> bytes:
    """
    RFC 5869 HKDF-SHA256 matching `sjcl.misc.hkdf` in `core.js`.
    """
    if length < 0 or length > 255 * 32:
        raise ValueError("Invalid HKDF output length")
    prk = hmac.new(salt, ikm, hashlib.sha256).digest()
    blocks = (length + 31) // 32
    okm = bytearray()
    prev = b""
    for i in range(1, blocks + 1):
        prev = hmac.new(prk, prev + info + bytes([i]), hashlib.sha256).digest()
        okm.extend(prev)
    return bytes(okm[:length])


def derive_mi_login_keys(
    mi_token_hex: str,
    mi_random_key_hex: str,
    mi_random_key_recv_hex: str,
) -> Tuple[str, str]:
    """
    Port of `do_login_generate()` in TelinkMiFlasher.html.

    Returns:
      (expected_device_infos_hex, mi_device_info_send_hex)
    """
    token_clean, _ = validate_mi_credentials(mi_token_hex)
    host_rand = bytes.fromhex(mi_random_key_hex)
    dev_rand = bytes.fromhex(mi_random_key_recv_hex)
    if len(host_rand) != 16:
        raise ValueError(f"Host random key must be 16 bytes, got {len(host_rand)}")
    if len(dev_rand) != 16:
        raise ValueError(f"Device random key must be 16 bytes, got {len(dev_rand)}")

    salt = host_rand + dev_rand
    salt_rev = dev_rand + host_rand
    derived = hkdf_sha256(
        ikm=bytes.fromhex(token_clean),
        length=64,
        salt=salt,
        info=b"mible-login-info",
    )
    expected_dev_info = hmac.new(derived[0:16], salt_rev, hashlib.sha256).hexdigest()
    send_dev_info = hmac.new(derived[16:32], salt, hashlib.sha256).hexdigest()
    return expected_dev_info, send_dev_info


def telink_crc32(data: bytes) -> int:
    """
    Port of `crc32()` in TelinkMiFlasher.html.

    IMPORTANT: TelinkMiFlasher's CRC32 starts with `crc = 0xFFFFFFFF` (-1) and
    returns `crc >>> 0` WITHOUT the standard final `^ 0xFFFFFFFF` inversion!
    Since Python's `zlib.crc32(data)` applies the final `^ 0xFFFFFFFF`, bitwise
    negation `(~zlib.crc32(data)) & 0xFFFFFFFF` recovers the exact un-inverted
    accumulator value.
    """
    return (~zlib.crc32(data)) & 0xFFFFFFFF


def crc16_modbus(data: bytes) -> int:
    """Port of `crc16_modbus()` in TelinkMiFlasher.html (poly 0xA001, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# ---------------------------------------------------------------------------
# Firmware Inspection, 2.1.1_0159 Signature Exploit & OTA Packet Builder
# ---------------------------------------------------------------------------

def inspect_ota_firmware(
    data: bytes,
    max_size: int = MAX_BLE_OTA_SIZE,
    big_ota_enabled: bool = False,
) -> FirmwareInfo:
    """
    Port of `testOTAFirmware()` in TelinkMiFlasher.html.

    Validates Telink OTA header, CRC32, size limits, optional Zigbee OTA header
    (0x0BEEF11E), and checks whether a Xiaomi MITFOTA signature is already appended.
    """
    fsize = len(data)
    if fsize < 1024:
        raise FlasherError(f"Wrong binary Telink OTA firmware size ({fsize} bytes < 1024)!")

    is_zb = False
    man_code: Optional[int] = None
    image_type: Optional[int] = None
    fw_ver: Optional[int] = None
    payload = data

    magic0 = struct.unpack_from("<I", data, 0)[0]
    if magic0 == ZIGBEE_OTA_MAGIC:
        hdr_ver, hdr_len = struct.unpack_from("<HH", data, 4)
        if hdr_ver != 0x0100 or hdr_len != 56:
            raise FlasherError("Unknown Zigbee OTA header!")
        zb_size = struct.unpack_from("<I", data, 0x3A)[0]
        if zb_size < 2048 or zb_size + 0x3E > fsize:
            raise FlasherError("Wrong binary Zigbee OTA firmware size!")
        man_code, image_type, fw_ver = struct.unpack_from("<HHI", data, 0x0A)
        is_zb = True
        if not big_ota_enabled:
            if 0x0203 <= image_type <= 0x0205:
                raise FlasherError(
                    f"Current firmware does not support OTA for Zigbee image type 0x{image_type:04x}."
                )
            if zb_size > max_size:
                raise FlasherError(
                    "Current firmware does not support EXT OTA. Flash custom BLE firmware v4.6+ first!"
                )
        elif zb_size > MAX_EXT_OTA_SIZE:
            raise FlasherError("Firmware size exceeds 208 KiB (MAX_EXT_OTA_SIZE)!")
        payload = data[0x3E : 0x3E + zb_size]
        fsize = len(payload)
    else:
        limit = MAX_EXT_OTA_SIZE if big_ota_enabled else max_size
        if fsize > limit:
            raise FlasherError(f"Firmware size ({fsize} bytes) exceeds maximum OTA size ({limit} bytes)!")

    if len(payload) < 0x20 or struct.unpack_from("<I", payload, 0x08)[0] != TELINK_MAGIC:
        raise FlasherError("Incorrect 'TLNK' header in Telink OTA binary firmware!")

    hsize = struct.unpack_from("<I", payload, 0x18)[0]
    if hsize > fsize or (hsize & 0x0F) != 4:
        raise FlasherError(
            f"Invalid size pointer (0x{hsize:x}) in Telink OTA binary firmware!"
        )

    calc_crc = telink_crc32(payload[: hsize - 4])
    file_crc = struct.unpack_from("<I", payload, hsize - 4)[0]
    if calc_crc != file_crc:
        raise FlasherError(
            f"Incorrect CRC in Telink OTA binary firmware (file=0x{file_crc:08x}, calc=0x{calc_crc:08x})!"
        )

    already_signed = False
    if hsize < fsize and (fsize - hsize) > 1024:
        if payload[hsize : hsize + 12] == MITFOTA_HEADER:
            already_signed = True

    return FirmwareInfo(
        payload=payload,
        header_size=hsize,
        file_crc=file_crc,
        already_signed=already_signed,
        is_zigbee_ota=is_zb,
        zigbee_man_code=man_code,
        zigbee_image_type=image_type,
        zigbee_fw_version=fw_ver,
    )


def append_signature_211_0159(firmware_payload: bytes, sign_data: bytes) -> bytes:
    """
    Exact port of `getSignFile()` in TelinkMiFlasher.html (lines 2690-2735).

    Appends the patched MITFOTA signature header and CRC-decoded exploit block
    from `sign_2.1.1_0159c.bin` to an unsigned Telink OTA binary.
    """
    if len(sign_data) < 64:
        raise FlasherError("Invalid signature file (smaller than 64 bytes)!")

    cdata = bytearray(sign_data)
    cl = cdata[0] | (cdata[1] << 8)
    if cl < 64 or cl > len(cdata):
        raise FlasherError(f"Invalid signature file header length (cl={cl}, total={len(cdata)})!")

    crc = telink_crc32(bytes(cdata[:cl]))
    fwsize = len(firmware_payload)
    struct.pack_into("<I", cdata, 0x14, fwsize)

    out = bytearray(firmware_payload)
    out.extend(b"\x00\x00\x00\x00")
    out.extend(cdata[4:cl])

    idx = cl
    if idx < len(cdata):
        if idx + 2 > len(cdata):
            raise FlasherError("Truncated fill descriptor in signature file!")
        x = cdata[idx] | (cdata[idx + 1] << 8)
        idx += 2
        fill_len = (crc ^ x) & 0xFFFF
        crc = (crc & 0xFFFF0000) | x
        xdata = bytearray([0xFF] * fill_len)

        while idx < len(cdata):
            if idx + 6 > len(cdata):
                raise FlasherError("Truncated patch record in signature file!")
            x = cdata[idx] | (cdata[idx + 1] << 8)
            idx += 2
            lx = (crc ^ x) & 0xFFFF
            crc = (crc & 0xFFFF0000) | x

            x = struct.unpack_from("<I", cdata, idx)[0]
            idx += 4
            crc = (crc ^ x) & 0xFFFFFFFF
            if lx + 4 > len(xdata):
                raise FlasherError(
                    f"Signature patch offset 0x{lx:04x} out of bounds (len=0x{len(xdata):04x})!"
                )
            struct.pack_into("<I", xdata, lx, crc)
            crc = x

        out.extend(xdata)

    return bytes(out)


def prepare_ota_image(
    raw_firmware: bytes,
    sign_data: Optional[bytes] = None,
    force_no_sign: bool = False,
    max_size: int = MAX_BLE_OTA_SIZE,
    big_ota_enabled: bool = False,
) -> Tuple[bytes, FirmwareInfo, bool]:
    """
    Validate firmware, optionally apply `sign_2.1.1_0159c.bin`, pad to 16-byte OTA
    blocks with 0xFF, and check final size limits.

    Returns:
      (padded_ota_bytes, fw_info, signature_was_appended)
    """
    info = inspect_ota_firmware(
        raw_firmware,
        max_size=max_size,
        big_ota_enabled=big_ota_enabled,
    )
    image = bytearray(info.payload)
    signed_applied = False

    if sign_data is not None and not force_no_sign and not info.already_signed:
        image = bytearray(append_signature_211_0159(bytes(image), sign_data))
        signed_applied = True

    remainder = len(image) % 16
    if remainder != 0:
        image.extend(b"\xff" * (16 - remainder))

    limit = MAX_EXT_OTA_SIZE if big_ota_enabled else max_size
    if len(image) > limit:
        raise FlasherError(
            f"Prepared OTA image ({len(image)} bytes) exceeds max allowed OTA size ({limit} bytes)!"
        )

    return bytes(image), info, signed_applied


def build_ota_block_packet(block_nr: int, block_16: bytes) -> bytes:
    """Build a 20-byte Telink OTA data packet: uint16_le(block_nr) + 16B data + uint16_le(crc16)."""
    if len(block_16) != 16:
        raise ValueError(f"OTA block must be exactly 16 bytes, got {len(block_16)}")
    body = struct.pack("<H", block_nr & 0xFFFF) + block_16
    crc = crc16_modbus(body)
    return body + struct.pack("<H", crc)


def build_ota_end_packet(block_count: int) -> bytes:
    """Port of `sendLastOTA()` packet in TelinkMiFlasher.html."""
    if block_count <= 0:
        raise ValueError("block_count must be > 0")
    last_idx = (block_count - 1) & 0xFFFF
    inv_idx = (~last_idx) & 0xFFFF
    return b"\x02\xff" + struct.pack("<HH", last_idx, inv_idx)


def map_lywsd03mmc_hw(hw_rev: Optional[str]) -> Tuple[int, int]:
    """Map Device Information 0x2a27 string ('B1.4', 'B1.5', 'B1.6', 'B1.1', ...) to (cfg.hver, hwver_id)."""
    prefix = (hw_rev or "")[:4]
    return LYWSD03MMC_HW_MAP.get(prefix, (0, 0))


# ---------------------------------------------------------------------------
# MiBLE v1 / v2 Login State Machine (Service 0xfe95: enc_10 & enc_19)
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class GattWriteAction:
    target: str  # "enc_10" or "enc_19"
    data: bytes
    delay_before_s: float = 0.0


class MiLoginSession:
    """
    Deterministic state machine implementing `sendLogin()` and the `enc_10` /
    `enc_19` notification handlers from `miAuthorization()` in TelinkMiFlasher.html.

    Supports BOTH:
      - MiBLE v2 extended-frame protocol (`000004xx` MTU negotiation + states
        `0 -> 1 -> 12 -> 13 -> 14`, used on firmware `2.1.1_0159`)
      - Classic MiBLE v1 chunked protocol (states `0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7`)
    """

    def __init__(self, mi_token_hex: str, host_random_bytes: Optional[bytes] = None) -> None:
        self.mi_token_hex, _ = validate_mi_credentials(mi_token_hex)
        self._fixed_random = host_random_bytes
        self.mi_random_key: str = ""
        self.mi_random_key_recv: str = ""
        self.mi_device_info_recv: str = ""
        self.mi_device_info_send: str = ""
        self.expected_device_infos: str = ""
        self.state: int = 0
        self.is_logged_in: bool = False
        self.error: Optional[str] = None

    def start_login(self) -> List[GattWriteAction]:
        """Generate fresh 16-byte random challenge and return initial `sendLogin()` writes."""
        rand_bytes = self._fixed_random if self._fixed_random is not None else secrets.token_bytes(16)
        if len(rand_bytes) != 16:
            raise ValueError("host_random_bytes must be 16 bytes")
        self.mi_random_key = rand_bytes.hex()
        self.state = 0
        self.is_logged_in = False
        self.error = None
        return [
            GattWriteAction("enc_10", bytes.fromhex("24000000")),
            GattWriteAction("enc_19", bytes.fromhex("0000000b0100")),
        ]

    def on_enc_10(self, payload: bytes) -> List[GattWriteAction]:
        """Handle notification on 0x0010 (`enc_10`)."""
        val = payload.hex()
        if val in ENC_10_STATUS_MESSAGES:
            msg, ok = ENC_10_STATUS_MESSAGES[val]
            if val in ("21000000", "e2000000"):
                self.is_logged_in = True
            elif not ok:
                self.error = msg
        return []

    def on_enc_19(self, payload: bytes) -> List[GattWriteAction]:
        """Handle notification on 0x0019 (`enc_19`) and return response writes."""
        val = payload.hex()
        actions: List[GattWriteAction] = []

        # Session / frame negotiation frames (used by 2.1.1_0159)
        if val.startswith("000004"):
            sub = val[6:8]
            rest = val[8:]
            if sub == "00":
                actions.append(GattWriteAction("enc_19", bytes.fromhex("00000500" + rest)))
                return actions
            if sub == "01":
                actions.append(GattWriteAction("enc_19", bytes.fromhex("00000501" + rest)))
                restart = self.start_login()
                if restart:
                    restart[0].delay_before_s = 0.25
                actions.extend(restart)
                return actions

        if self.state == 0 and val == "00000101":
            self.state = 1
            actions.append(GattWriteAction("enc_19", bytes.fromhex("0100" + self.mi_random_key)))

        # --- Classic MiBLE v1 chunked path (states 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7) ---
        elif self.state == 1 and val == "0000000d0100":
            self.state = 2
            actions.append(GattWriteAction("enc_19", bytes.fromhex("00000101")))

        elif self.state == 2 and val.startswith("0100"):
            self.state = 3
            self.mi_random_key_recv = val[4:]
            self.expected_device_infos, self.mi_device_info_send = derive_mi_login_keys(
                self.mi_token_hex, self.mi_random_key, self.mi_random_key_recv
            )
            actions.append(GattWriteAction("enc_19", bytes.fromhex("00000100")))

        elif self.state == 3 and val == "0000000c0200":
            self.state = 4
            actions.append(GattWriteAction("enc_19", bytes.fromhex("00000101")))

        elif self.state == 4 and val.startswith("0100"):
            self.state = 5
            self.mi_device_info_recv = val[4:]

        elif self.state == 5 and val.startswith("0200"):
            self.state = 6
            self.mi_device_info_recv += val[4:]
            if self.expected_device_infos != self.mi_device_info_recv:
                self.state = 0
                self.error = (
                    "Received device auth digest does not match expected HMAC — "
                    "verify that your Mi Token matches this device!"
                )
                actions.append(GattWriteAction("enc_19", bytes.fromhex("00000100")))
            else:
                actions.append(GattWriteAction("enc_19", bytes.fromhex("00000100")))
                actions.append(GattWriteAction("enc_19", bytes.fromhex("0000000a0200")))

        elif self.state == 6 and val == "00000101":
            self.state = 7
            actions.append(
                GattWriteAction("enc_19", bytes.fromhex("0100" + self.mi_device_info_send[0:36]))
            )
            actions.append(
                GattWriteAction("enc_19", bytes.fromhex("0200" + self.mi_device_info_send[36:72]))
            )

        # --- Extended-MTU MiBLE v2 path (states 1 -> 12 -> 13 -> 14, used by 2.1.1_0159) ---
        elif self.state == 1 and val.startswith("0000020d"):
            self.state = 12
            self.mi_random_key_recv = val[8:]
            self.expected_device_infos, self.mi_device_info_send = derive_mi_login_keys(
                self.mi_token_hex, self.mi_random_key, self.mi_random_key_recv
            )
            actions.append(GattWriteAction("enc_19", bytes.fromhex("00000300")))

        elif self.state == 12 and val.startswith("0000020c"):
            self.state = 13
            self.mi_device_info_recv = val[8:]
            actions.append(GattWriteAction("enc_19", bytes.fromhex("00000300")))
            if self.expected_device_infos == self.mi_device_info_recv:
                actions.append(GattWriteAction("enc_19", bytes.fromhex("0000000a0100")))
            else:
                self.state = 0
                self.error = (
                    "Received device auth digest does not match expected HMAC — "
                    "verify that your Mi Token matches this device!"
                )
                actions.append(GattWriteAction("enc_19", bytes.fromhex("00000100")))

        elif self.state == 13 and val == "00000101":
            self.state = 14
            actions.append(GattWriteAction("enc_19", bytes.fromhex("0100" + self.mi_device_info_send)))

        elif self.state == 14 and val == "00000100":
            self.state = 0

        elif val == "000001050100":
            self.error = "Received login timeout from device (000001050100)."

        return actions


# ---------------------------------------------------------------------------
# Xiaomi Cloud Token Extractor File Parser (JSON & Console Output)
# ---------------------------------------------------------------------------

def parse_tokens_file(path: Path) -> List[MiCredentials]:
    """
    Parse credentials from `Xiaomi-cloud-tokens-extractor`:
      1. `-o tokens.json` structured JSON output (`[{"server": ..., "homes": [{"devices": [...]}]}]`)
      2. Flat JSON list or dict of device objects
      3. Saved stdout text from `token_extractor.py` (with or without ANSI color codes)
    """
    raw_text = path.read_text(encoding="utf-8", errors="replace").strip()
    if not raw_text:
        return []

    # Try JSON first
    try:
        parsed = json.loads(raw_text)
        return _extract_creds_from_json(parsed)
    except json.JSONDecodeError:
        pass

    # Fallback: parse token_extractor.py console text output
    return _extract_creds_from_text(raw_text)


def _extract_creds_from_json(data: Any) -> List[MiCredentials]:
    results: List[MiCredentials] = []
    seen_macs: set[str] = set()

    def visit_device(dev: Dict[str, Any]) -> None:
        token = str(dev.get("token") or dev.get("mi_token") or "").strip()
        mac_raw = str(dev.get("mac") or "").strip()
        ble_data = dev.get("BLE_DATA") if isinstance(dev.get("BLE_DATA"), dict) else {}
        bind_key = str(
            ble_data.get("beaconkey")
            or dev.get("beaconkey")
            or dev.get("bind_key")
            or dev.get("mi_bind_key")
            or ""
        ).strip() or None
        if not token and not mac_raw:
            return
        mac_norm: Optional[str] = None
        if mac_raw:
            try:
                mac_norm = normalize_mac(mac_raw)
            except ValueError:
                mac_norm = mac_raw
        key_id = mac_norm or str(dev.get("did") or "")
        if key_id and key_id in seen_macs:
            return
        if key_id:
            seen_macs.add(key_id)
        results.append(
            MiCredentials(
                mac=mac_norm,
                name=dev.get("name"),
                did=dev.get("did") or dev.get("known_id"),
                token=token,
                bind_key=bind_key,
                model=dev.get("model"),
            )
        )

    if isinstance(data, dict):
        if "devices" in data and isinstance(data["devices"], list):
            for d in data["devices"]:
                if isinstance(d, dict):
                    visit_device(d)
        else:
            visit_device(data)
    elif isinstance(data, list):
        for entry in data:
            if not isinstance(entry, dict):
                continue
            if "homes" in entry and isinstance(entry["homes"], list):
                for home in entry["homes"]:
                    if isinstance(home, dict) and isinstance(home.get("devices"), list):
                        for d in home["devices"]:
                            if isinstance(d, dict):
                                visit_device(d)
            else:
                visit_device(entry)

    return results


def _extract_creds_from_text(text: str) -> List[MiCredentials]:
    clean = ANSI_ESCAPE_RE.sub("", text)
    blocks = re.split(r"-{5,}", clean)
    results: List[MiCredentials] = []
    seen: set[str] = set()

    for block in blocks:
        fields: Dict[str, str] = {}
        for line in block.splitlines():
            m = re.match(r"^\s*(NAME|ID|BLE KEY|MAC|TOKEN|MODEL)\s*:\s*(.+?)\s*$", line)
            if m:
                fields[m.group(1)] = m.group(2).strip()
        if not fields.get("TOKEN") and not fields.get("MAC"):
            continue
        mac_norm: Optional[str] = None
        if "MAC" in fields:
            try:
                mac_norm = normalize_mac(fields["MAC"])
            except ValueError:
                mac_norm = fields["MAC"]
        dedup = mac_norm or fields.get("ID", "")
        if dedup and dedup in seen:
            continue
        if dedup:
            seen.add(dedup)
        results.append(
            MiCredentials(
                mac=mac_norm,
                name=fields.get("NAME"),
                did=fields.get("ID"),
                token=fields.get("TOKEN", ""),
                bind_key=fields.get("BLE KEY"),
                model=fields.get("MODEL"),
            )
        )
    return results


def lookup_credentials_for_mac(creds: Sequence[MiCredentials], mac: str) -> Optional[MiCredentials]:
    target = normalize_mac(mac)
    for item in creds:
        if item.mac and item.mac.upper() == target:
            return item
    return None


# ---------------------------------------------------------------------------
# Remote Asset Fetching & Local Cache (firmware.json, ATC_vNN.bin, signfile)
# ---------------------------------------------------------------------------

def default_cache_dir() -> Path:
    xdg = os.environ.get("XDG_CACHE_HOME")
    base = Path(xdg) if xdg else (Path.home() / ".cache")
    return base / "telink_mi_flasher"


def fetch_repo_asset(
    rel_or_url: str,
    cache_dir: Optional[Path] = None,
    refresh: bool = False,
    base_url: str = RAW_REPO_BASE_URL,
    timeout_s: float = 15.0,
) -> bytes:
    """Download an asset from pvvx/ATC_MiThermometer with optional local file caching."""
    if rel_or_url.startswith(("http://", "https://")):
        url = rel_or_url
        cache_rel = Path(urllib.request.urlparse(url).path.lstrip("/"))
    else:
        rel_clean = rel_or_url.lstrip("/")
        url = base_url.rstrip("/") + "/" + rel_clean
        cache_rel = Path(rel_clean)

    cache_path: Optional[Path] = None
    if cache_dir is not None:
        cache_path = cache_dir / cache_rel
        if cache_path.is_file() and not refresh and rel_or_url != "firmware.json":
            return cache_path.read_bytes()

    req = urllib.request.Request(url, headers={"User-Agent": "telink-mi-flasher-linux/1.0"})
    try:
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            data = resp.read()
    except urllib.error.URLError as exc:
        if cache_path is not None and cache_path.is_file():
            return cache_path.read_bytes()
        raise FlasherError(f"Failed to download '{url}': {exc}") from exc

    if cache_path is not None:
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        cache_path.write_bytes(data)

    return data


def resolve_upstream_firmware_path(
    firmware_index: Dict[str, Any],
    hwver_id: int,
    variant: str = "custom",
) -> str:
    """
    Resolve firmware relative path from pvvx `firmware.json` for a given `hwver_id`.

    `variant`:
      - `"custom"`: latest stable pvvx custom BLE firmware (e.g. `bin/ATC_v59.bin`)
      - `"signed-original"`: Xiaomi-signed 2.1.1_0159c (`bin/Original/Original_OTA_Xiaomi_LYWSD03MMC_v2.1.1_0159c.bin`)
      - `"original"`: stock rollback firmware (`bin/Original/...`)
    """
    key_map = {
        "custom": "custom",
        "signed-original": "signed",
        "original": "original",
    }
    json_key = key_map.get(variant)
    if not json_key or json_key not in firmware_index:
        raise FlasherError(f"Firmware variant '{variant}' not available in firmware.json")

    entries = firmware_index[json_key]
    if not isinstance(entries, list) or hwver_id < 0 or hwver_id >= len(entries):
        raise FlasherError(
            f"Hardware ID {hwver_id} not covered by '{json_key}' table in firmware.json"
        )
    fn = entries[hwver_id]
    if not fn or fn in ("?", "/"):
        raise FlasherError(
            f"No '{variant}' firmware entry available for hwver_id={hwver_id}"
        )
    return str(fn)


# ---------------------------------------------------------------------------
# BLE Transport & Flasher Orchestration (Linux BlueZ via Bleak)
# ---------------------------------------------------------------------------

def _import_bleak() -> Tuple[Any, Any]:
    try:
        from bleak import BleakClient, BleakScanner  # type: ignore
        return BleakClient, BleakScanner
    except ImportError as exc:
        raise FlasherError(
            "Missing Python BLE library 'bleak'. Install it on Linux with:\n"
            "  pip install bleak\n"
            "or run with uv:\n"
            "  uv run tools/telink_mi_flasher.py ..."
        ) from exc


async def write_gatt_auto(client: Any, char: Any, data: bytes) -> None:
    """
    Mimic Web Bluetooth `characteristic.writeValue(data)` on BlueZ:
    use Write-With-Response if the characteristic supports `"write"`, otherwise
    fall back to `"write-without-response"`.
    """
    props = set(getattr(char, "properties", ()) or ())
    use_response = "write" in props or "write-without-response" not in props
    await client.write_gatt_char(char, data, response=use_response)


def _get_service_char(client: Any, service_uuid: str, char_uuid: str) -> Optional[Any]:
    svc = client.services.get_service(service_uuid)
    if svc is None:
        return None
    return svc.get_characteristic(char_uuid)


async def _read_utf8_char(client: Any, service_uuid: str, char_uuid: str) -> Optional[str]:
    char = _get_service_char(client, service_uuid, char_uuid)
    if char is None:
        return None
    try:
        raw = await client.read_gatt_char(char)
        return bytes(raw).decode("utf-8", errors="replace").rstrip("\x00").strip()
    except Exception:
        return None


async def probe_device_profile(
    client: Any,
    address: str,
    adv_name: Optional[str] = None,
) -> DeviceProfile:
    """Inspect GATT services and Device Information (0x180a) matching `doConnect()`."""
    svc_uuids = {s.uuid.lower() for s in client.services}
    ota_enabled = UUID_SVC_TELINK_OTA in svc_uuids
    mi_enabled = UUID_SVC_MI_MAIN in svc_uuids
    mi_auth_enabled = UUID_SVC_MI_AUTH in svc_uuids
    custom_enabled = UUID_SVC_CUSTOM in svc_uuids
    bthome_enabled = UUID_SVC_BTHOME in svc_uuids
    dev_inf_enabled = UUID_SVC_DEV_INFO in svc_uuids

    fw_rev: Optional[str] = None
    hw_rev: Optional[str] = None
    sw_rev: Optional[str] = None
    if dev_inf_enabled:
        fw_rev = await _read_utf8_char(client, UUID_SVC_DEV_INFO, UUID_CHR_FW_REV)
        hw_rev = await _read_utf8_char(client, UUID_SVC_DEV_INFO, UUID_CHR_HW_REV)
        sw_rev = await _read_utf8_char(client, UUID_SVC_DEV_INFO, UUID_CHR_SW_REV)

    # BlueZ often sets `dev.name` to a hyphenated MAC alias ("A4-C1-38-XX-XX-XX")
    # when passive scanning has not received a Scan Response yet; prefer GAP 0x2a00 if so.
    is_mac_alias = bool(
        adv_name and re.fullmatch(r"[0-9A-Fa-f]{2}([-:][0-9A-Fa-f]{2}){5}", adv_name.strip())
    )
    dev_name = "" if is_mac_alias else (adv_name or "")
    if not dev_name and UUID_SVC_GAP in svc_uuids:
        gap_name = await _read_utf8_char(client, UUID_SVC_GAP, UUID_CHR_GAP_NAME)
        if gap_name:
            dev_name = gap_name
    if not dev_name and mi_enabled and not custom_enabled:
        if (hw_rev or "")[:4] in LYWSD03MMC_HW_MAP or (fw_rev or "").startswith("2.1.1_0159"):
            dev_name = "LYWSD03MMC"

    hwver_id: Optional[int] = None
    big_ota_enabled = False
    custom_ver: Optional[int] = None

    if dev_name.startswith("LYWSD03MMC") or ((hw_rev or "")[:4] in LYWSD03MMC_HW_MAP):
        _, hwver_id = map_lywsd03mmc_hw(hw_rev)

    # If running custom pvvx firmware (0x1f10), query 0x55 config block for exact hwver_id & bigOta
    pvvx_config: Optional[PvvxConfig] = None
    if custom_enabled:
        cfg_char = _get_service_char(client, UUID_SVC_CUSTOM, UUID_CHR_CUSTOM_CFG)
        if cfg_char is not None and "notify" in (cfg_char.properties or []):
            cfg_evt: asyncio.Queue[bytes] = asyncio.Queue()

            def _on_cfg(_sender: Any, data: bytearray) -> None:
                cfg_evt.put_nowait(bytes(data))

            try:
                await client.start_notify(cfg_char, _on_cfg)
                await write_gatt_auto(client, cfg_char, b"\x55")
                pkt = await asyncio.wait_for(cfg_evt.get(), timeout=2.5)
                if len(pkt) >= 10 and pkt[0] == 0x55:
                    pvvx_config = PvvxConfig.from_bytes(pkt)
                    custom_ver = pvvx_config.ver
                    hwver_id = pvvx_config.hwver_id
                    if custom_ver > 0x45:
                        big_ota_enabled = True
            except Exception:
                pass
            finally:
                try:
                    await client.stop_notify(cfg_char)
                except Exception:
                    pass

    # Optionally capture one Mi temperature/humidity/battery sample if Mi service is active
    temp_c: Optional[float] = None
    humi_pct: Optional[int] = None
    bat_mv: Optional[int] = None
    if mi_enabled:
        temp_char = _get_service_char(client, UUID_SVC_MI_MAIN, UUID_CHR_MI_TEMP)
        if temp_char is not None and "notify" in (temp_char.properties or []):
            sample_q: asyncio.Queue[bytes] = asyncio.Queue()

            def _on_temp(_sender: Any, data: bytearray) -> None:
                sample_q.put_nowait(bytes(data))

            try:
                await client.start_notify(temp_char, _on_temp)
                raw = await asyncio.wait_for(sample_q.get(), timeout=2.5)
                if len(raw) >= 5:
                    temp_raw, humi_raw, vbat_raw = struct.unpack_from("<hBH", raw, 0)
                    temp_c = temp_raw / 100.0
                    humi_pct = int(humi_raw)
                    bat_mv = int(vbat_raw)
            except Exception:
                pass
            finally:
                try:
                    await client.stop_notify(temp_char)
                except Exception:
                    pass

    return DeviceProfile(
        address=address,
        name=dev_name or "Unknown",
        fw_rev=fw_rev,
        hw_rev=hw_rev,
        sw_rev=sw_rev,
        ota_enabled=ota_enabled,
        mi_enabled=mi_enabled,
        mi_auth_enabled=mi_auth_enabled,
        custom_enabled=custom_enabled,
        bthome_enabled=bthome_enabled,
        big_ota_enabled=big_ota_enabled,
        hwver_id=hwver_id,
        custom_ver=custom_ver,
        temperature_c=temp_c,
        humidity_pct=humi_pct,
        battery_mv=bat_mv,
        pvvx_config=pvvx_config,
    )


async def perform_mi_login(
    client: Any,
    mi_token_hex: str,
    timeout_s: float = 20.0,
    log: Callable[[str], None] = print,
) -> None:
    """
    Execute the full MiBLE authentication handshake over service `0xfe95`
    (`enc_10` = `0x0010`, `enc_19` = `0x0019`) using serialized GATT writes.
    """
    enc_10 = _get_service_char(client, UUID_SVC_MI_AUTH, UUID_CHR_MI_ENC_10)
    enc_19 = _get_service_char(client, UUID_SVC_MI_AUTH, UUID_CHR_MI_ENC_19)
    if enc_10 is None or enc_19 is None:
        raise FlasherError("Mi authentication characteristics (0xfe95 / 0x0010 / 0x0019) not found!")

    char_map = {"enc_10": enc_10, "enc_19": enc_19}
    event_q: asyncio.Queue[Tuple[str, bytes]] = asyncio.Queue()
    session = MiLoginSession(mi_token_hex)

    def _on_enc_10(_sender: Any, data: bytearray) -> None:
        event_q.put_nowait(("enc_10", bytes(data)))

    def _on_enc_19(_sender: Any, data: bytearray) -> None:
        event_q.put_nowait(("enc_19", bytes(data)))

    await client.start_notify(enc_10, _on_enc_10)
    await client.start_notify(enc_19, _on_enc_19)
    try:
        log("Starting Mi Token authentication handshake (0xfe95)...")
        for act in session.start_login():
            if act.delay_before_s > 0:
                await asyncio.sleep(act.delay_before_s)
            await write_gatt_auto(client, char_map[act.target], act.data)

        deadline = time.monotonic() + timeout_s
        while not session.is_logged_in:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise FlasherError(
                    f"Mi Login timed out after {timeout_s:.1f}s (last state={session.state})."
                )
            src, pkt = await asyncio.wait_for(event_q.get(), timeout=remaining)
            if src == "enc_10":
                followups = session.on_enc_10(pkt)
            else:
                followups = session.on_enc_19(pkt)

            for act in followups:
                if act.delay_before_s > 0:
                    await asyncio.sleep(act.delay_before_s)
                await write_gatt_auto(client, char_map[act.target], act.data)

            if session.error:
                raise FlasherError(session.error)

        log("Mi Login successful!")
    finally:
        for ch in (enc_19, enc_10):
            try:
                await client.stop_notify(ch)
            except Exception:
                pass


async def stream_telink_ota(
    client: Any,
    ota_image: bytes,
    mi_enabled: bool = False,
    block_delay_s: float = 0.002,
    post_end_wait_s: float = 2.0,
    show_progress: bool = True,
    log: Callable[[str], None] = print,
) -> float:
    """
    Port of `updateBegin()`, `sendOTAblock()`, and `sendLastOTA()` in TelinkMiFlasher.html.
    """
    if len(ota_image) == 0 or len(ota_image) % 16 != 0:
        raise ValueError("ota_image must be non-empty and padded to a 16-byte boundary.")

    ota_char = _get_service_char(client, UUID_SVC_TELINK_OTA, UUID_CHR_TELINK_OTA)
    if ota_char is None:
        raise FlasherError("Telink OTA write characteristic (00010203-...-2b12) not found!")

    # Request fast connection interval on Xiaomi firmware (`1e0000` -> MiSpeed characteristic)
    if mi_enabled:
        speed_char = _get_service_char(client, UUID_SVC_MI_MAIN, UUID_CHR_MI_SPEED)
        if speed_char is not None:
            try:
                await write_gatt_auto(client, speed_char, bytes.fromhex("1e0000"))
            except Exception:
                pass

    await asyncio.sleep(0.5)

    props = set(getattr(ota_char, "properties", ()) or ())
    ota_write_with_resp = "write" in props or "write-without-response" not in props

    # Telink OTA start sequence: 0x00ff (OTA version), 0x01ff (OTA start), 300ms pause
    await client.write_gatt_char(ota_char, b"\x00\xff", response=ota_write_with_resp)
    await client.write_gatt_char(ota_char, b"\x01\xff", response=ota_write_with_resp)
    await asyncio.sleep(0.3)

    block_count = len(ota_image) // 16
    start_ts = time.monotonic()
    last_print_pct = -1

    for block_nr in range(block_count):
        chunk = ota_image[block_nr * 16 : block_nr * 16 + 16]
        pkt = build_ota_block_packet(block_nr, chunk)
        await client.write_gatt_char(ota_char, pkt, response=ota_write_with_resp)

        if (block_nr + 1) % 8 == 0:
            # Synchronous GATT Read every 8 blocks (128 bytes): drains queued WriteWithoutResponse
            # PDUs on BlueZ and checks the device's 1-byte Telink OTA status register.
            status_bytes = await client.read_gatt_char(ota_char)
            if len(status_bytes) > 0 and status_bytes[0] != 0:
                code = status_bytes[0]
                reason = OTA_ERROR_TABLE[code] if code < len(OTA_ERROR_TABLE) else f"Unknown ({code})"
                raise FlasherError(f"Device reported OTA error at block {block_nr}: {reason}")
        elif not ota_write_with_resp and block_delay_s > 0:
            await asyncio.sleep(block_delay_s)

        if show_progress:
            pct = int(((block_nr + 1) * 100) // block_count)
            if pct != last_print_pct and (pct % 5 == 0 or block_nr + 1 == block_count):
                last_print_pct = pct
                elapsed = time.monotonic() - start_ts
                rate_kbs = ((block_nr + 1) * 16 / 1024.0) / max(elapsed, 0.001)
                print(
                    f"\r  OTA progress: {pct:3d}% ({block_nr + 1}/{block_count} blocks, "
                    f"{elapsed:5.1f}s, {rate_kbs:4.2f} KiB/s)",
                    end="",
                    flush=True,
                )

    if show_progress:
        print(flush=True)

    # Send Telink OTA end packet (0x02ff + last_block + ~last_block)
    end_pkt = build_ota_end_packet(block_count)
    try:
        await client.write_gatt_char(ota_char, end_pkt, response=ota_write_with_resp)
    except Exception as exc:
        # On 2.1.1_0159 signature exploit, the MCU may immediately reboot upon processing 02ff
        if not getattr(client, "is_connected", True):
            log("Device disconnected immediately on 02ff (rebooting into new firmware).")
        else:
            raise FlasherError(f"Failed sending final OTA block: {exc}") from exc

    # Give BlueZ HCI time to flush the final WriteWithoutResponse PDU over the air before disconnect
    steps = max(int(round(post_end_wait_s / 0.1)), 0)
    for _ in range(steps):
        if not getattr(client, "is_connected", True):
            break
        await asyncio.sleep(0.1)

    total_s = time.monotonic() - start_ts
    log(f"OTA transfer complete ({block_count} blocks, {len(ota_image)} bytes in {total_s:.1f}s).")
    return total_s


async def connect_with_retries(
    mac: str,
    adapter: Optional[str] = None,
    scan_timeout_s: float = 12.0,
    connect_timeout_s: float = 15.0,
    retries: int = 5,
    log: Callable[[str], None] = print,
) -> Tuple[Any, Optional[str]]:
    """
    Scan for `mac` via BlueZ (`BleakScanner`) and establish a `BleakClient` GATT
    connection with up to `retries` attempts (matching `connectTrys < 5` in TelinkMiFlasher.html).
    """
    BleakClient, BleakScanner = _import_bleak()
    target_mac = normalize_mac(mac)
    scanner_kwargs: Dict[str, Any] = {}
    if adapter:
        scanner_kwargs["adapter"] = adapter

    last_err: Optional[Exception] = None
    for attempt in range(1, retries + 1):
        try:
            log(f"[{attempt}/{retries}] Scanning for {target_mac}...")
            dev = await BleakScanner.find_device_by_address(
                target_mac, timeout=scan_timeout_s, **scanner_kwargs
            )
            if dev is None:
                raise FlasherError(
                    f"Device {target_mac} not found during {scan_timeout_s:.0f}s BLE scan."
                )
            adv_name = getattr(dev, "name", None)
            log(f"Connecting to {target_mac} ({adv_name or 'unnamed'})...")
            client = BleakClient(dev, timeout=connect_timeout_s, **scanner_kwargs)
            await client.connect()
            return client, adv_name
        except Exception as exc:
            last_err = exc
            err_msg = str(exc) or type(exc).__name__
            if attempt < retries:
                log(f"  Connection attempt {attempt} failed ({err_msg}); retrying in 2s...")
                await asyncio.sleep(2.0)

    last_msg = str(last_err) or (type(last_err).__name__ if last_err else "unknown error")
    raise FlasherError(f"Unable to connect to {target_mac} after {retries} attempts: {last_msg}")


# ---------------------------------------------------------------------------
# High-Level Workflows (`scan`, `info`, `flash`, `build-image`, `list-tokens`)
# ---------------------------------------------------------------------------

async def run_scan(
    timeout_s: float,
    adapter: Optional[str] = None,
    name_filter: Optional[str] = None,
    known_creds: Optional[Sequence[MiCredentials]] = None,
) -> int:
    _, BleakScanner = _import_bleak()
    kwargs: Dict[str, Any] = {}
    if adapter:
        kwargs["adapter"] = adapter

    print(f"Scanning BLE devices for {timeout_s:.1f}s...")
    discovered = await BleakScanner.discover(timeout=timeout_s, return_adv=True, **kwargs)
    cred_by_mac = {c.mac: c for c in (known_creds or []) if c.mac}

    rows: List[Tuple[int, str, str, str]] = []
    for _addr, (dev, adv) in discovered.items():
        mac = (getattr(dev, "address", "") or "").upper()
        name = getattr(dev, "name", None) or getattr(adv, "local_name", None) or ""
        rssi = getattr(adv, "rssi", None)
        rssi_val = int(rssi) if rssi is not None else -999

        if name_filter:
            prefixes = [p.strip().lower() for p in name_filter.split(",") if p.strip()]
            if prefixes and not any(name.lower().startswith(p) for p in prefixes) and mac not in cred_by_mac:
                continue

        note = ""
        if mac in cred_by_mac:
            c = cred_by_mac[mac]
            note = f"token=yes ({c.name or c.model or 'in tokens-file'})"
        rows.append((rssi_val, mac, name or "<unnamed>", note))

    rows.sort(key=lambda r: r[0], reverse=True)
    if not rows:
        print("No matching BLE devices found.")
        return 1

    print(f"{'MAC Address':<19} {'RSSI':>6}  {'BLE Name':<22} {'Tokens File Match'}")
    print("-" * 74)
    for rssi_val, mac, name, note in rows:
        rssi_str = f"{rssi_val} dBm" if rssi_val > -999 else "   ?  "
        print(f"{mac:<19} {rssi_str:>6}  {name:<22} {note}")
    return 0


def format_device_profile(profile: DeviceProfile) -> str:
    lines = [
        f"Device:            {profile.name} ({profile.address})",
        f"Firmware Revision: {profile.fw_rev or 'N/A'}",
        f"Hardware Revision: {profile.hw_rev or 'N/A'}"
        + (f" (hwver_id={profile.hwver_id})" if profile.hwver_id is not None else ""),
        f"Software Revision: {profile.sw_rev or 'N/A'}",
        f"Telink OTA Svc:    {'yes' if profile.ota_enabled else 'NO'}",
        f"Mode:              "
        + (
            f"Custom pvvx/ATC firmware v{(profile.custom_ver >> 4)}.{(profile.custom_ver & 0x0F)}"
            if profile.custom_enabled and profile.custom_ver is not None
            else (
                "Custom pvvx/ATC firmware"
                if profile.custom_enabled
                else ("Xiaomi MiBLE stock firmware" if profile.mi_enabled else "Unknown")
            )
        ),
    ]
    if profile.temperature_c is not None:
        lines.append(
            f"Live Telemetry:    {profile.temperature_c:.2f} °C, "
            f"{profile.humidity_pct}% rH, {profile.battery_mv} mV"
        )
    if profile.pvvx_config is not None:
        lines.append("Custom Config (0x55):")
        lines.append(profile.pvvx_config.format_summary())
    if profile.is_fw_211_0159:
        lines.append(
            f"2.1.1_0159 Notes:  Requires Mi Home registration + 24-hex Mi Token login + "
            f"'{SIGN_FILE_211_0159}'."
        )
        if profile.is_2025_hw_variant:
            lines.append(
                f"2025 HW Notice:    Hardware '{profile.hw_rev}' may require '--two-stage' "
                "(flashes signed Original 2.1.1_0159c first, then custom ATC_vNN.bin)."
            )
    return "\n".join(lines)


async def run_info(
    mac: str,
    adapter: Optional[str] = None,
    retries: int = 5,
    scan_timeout_s: float = 12.0,
) -> int:
    client, adv_name = await connect_with_retries(
        mac=mac, adapter=adapter, scan_timeout_s=scan_timeout_s, retries=retries
    )
    try:
        profile = await probe_device_profile(client, normalize_mac(mac), adv_name)
        print(format_device_profile(profile))
    finally:
        if getattr(client, "is_connected", False):
            await client.disconnect()
    return 0


async def flash_single_stage(
    mac: str,
    mi_token: Optional[str],
    firmware_path: Optional[Path],
    sign_file_path: Optional[Path],
    variant: str,
    force_no_sign: bool,
    cache_dir: Path,
    refresh_cache: bool,
    adapter: Optional[str],
    retries: int,
    scan_timeout_s: float,
    block_delay_s: float,
) -> DeviceProfile:
    """Connect, authenticate if needed, build/sign OTA image, and stream OTA to `mac`."""
    client, adv_name = await connect_with_retries(
        mac=mac,
        adapter=adapter,
        scan_timeout_s=scan_timeout_s,
        retries=retries,
    )
    try:
        profile = await probe_device_profile(client, normalize_mac(mac), adv_name)
        print(format_device_profile(profile))

        if not profile.ota_enabled:
            raise FlasherError(
                f"Device {profile.address} does not expose Telink OTA service ({UUID_SVC_TELINK_OTA})!"
            )

        # Resolve raw firmware bytes (local file OR upstream pvvx repository)
        if firmware_path is not None:
            raw_fw = firmware_path.read_bytes()
            fw_label = str(firmware_path)
        else:
            hw_id = profile.hwver_id if profile.hwver_id is not None else 0
            print("Loading upstream firmware index ('firmware.json')...")
            fw_index = json.loads(
                fetch_repo_asset("firmware.json", cache_dir=cache_dir, refresh=refresh_cache).decode("utf-8")
            )
            rel_fw = resolve_upstream_firmware_path(fw_index, hw_id, variant=variant)
            print(f"Fetching firmware '{rel_fw}'...")
            raw_fw = fetch_repo_asset(rel_fw, cache_dir=cache_dir, refresh=refresh_cache)
            fw_label = rel_fw

        # Determine whether 2.1.1_0159c signature block is required
        sign_bytes: Optional[bytes] = None
        need_sign = (
            not force_no_sign
            and variant != "signed-original"
            and (sign_file_path is not None or profile.required_sign_file is not None)
        )
        if need_sign:
            if sign_file_path is not None:
                sign_bytes = sign_file_path.read_bytes()
                sign_label = str(sign_file_path)
            else:
                rel_sign = f"signfiles/{profile.required_sign_file}"
                print(f"Fetching signature file '{rel_sign}'...")
                sign_bytes = fetch_repo_asset(rel_sign, cache_dir=cache_dir, refresh=refresh_cache)
                sign_label = rel_sign
        else:
            sign_label = None

        max_ota = MAX_EXT_OTA_SIZE if profile.big_ota_enabled else MAX_BLE_OTA_SIZE
        ota_image, fw_info, sig_appended = prepare_ota_image(
            raw_firmware=raw_fw,
            sign_data=sign_bytes,
            force_no_sign=force_no_sign,
            max_size=max_ota,
            big_ota_enabled=profile.big_ota_enabled,
        )

        print(
            f"Prepared OTA payload from '{fw_label}': "
            f"raw={len(fw_info.payload)} B, crc32=0x{fw_info.file_crc:08x}, "
            f"signed={'appended (' + str(sign_label) + ')' if sig_appended else ('pre-signed MITFOTA' if fw_info.already_signed else 'no')}, "
            f"final={len(ota_image)} B ({len(ota_image) // 16} blocks)"
        )

        # Authenticate via Mi Token if device is running stock Xiaomi firmware
        if profile.mi_enabled and profile.mi_auth_enabled and not profile.custom_enabled:
            if not mi_token:
                raise FlasherError(
                    "Device is running Xiaomi firmware and requires a 24-hex Mi Token! "
                    "Pass --token <24hex> or --tokens-file <tokens.json>."
                )
            await perform_mi_login(client, mi_token_hex=mi_token)

        # Clear Ext.OTA flash region first if flashing >128KB or Zigbee OTA on custom firmware v4.6+
        if profile.custom_enabled and (fw_info.is_zigbee_ota or len(ota_image) > MAX_BLE_OTA_SIZE):
            cfg_char = _get_service_char(client, UUID_SVC_CUSTOM, UUID_CHR_CUSTOM_CFG)
            if not profile.big_ota_enabled or cfg_char is None:
                raise FlasherError(
                    "Extended OTA size requires custom BLE firmware v4.6 or higher!"
                )
            szk = (len(ota_image) + 1023) >> 10
            if (profile.custom_ver or 0) <= 0x53 and szk <= 128:
                szk = 129
            print(f"Clearing Extended OTA flash region ({szk} KiB)...")
            cmd = bytes([0x73, 0, 0, 4, 0]) + struct.pack("<I", szk)
            await write_gatt_auto(client, cfg_char, cmd)
            await asyncio.sleep(2.5)

        if profile.is_fw_211_0159 and profile.is_2025_hw_variant and sig_appended:
            print(
                f"NOTE: Hardware '{profile.hw_rev}' detected on 2.1.1_0159. If the sensor remains on "
                "stock Xiaomi firmware after reboot, re-run with '--two-stage'."
            )

        await stream_telink_ota(
            client=client,
            ota_image=ota_image,
            mi_enabled=profile.mi_enabled,
            block_delay_s=block_delay_s,
        )
        return profile
    finally:
        if getattr(client, "is_connected", False):
            try:
                await client.disconnect()
            except Exception:
                pass


async def run_flash_workflow(
    mac: str,
    mi_token: Optional[str],
    firmware_path: Optional[Path],
    sign_file_path: Optional[Path],
    signed_original: bool,
    two_stage: bool,
    force_no_sign: bool,
    reboot_wait_s: float,
    cache_dir: Path,
    refresh_cache: bool,
    adapter: Optional[str],
    retries: int,
    scan_timeout_s: float,
    block_delay_s: float,
) -> None:
    if two_stage:
        print(f"=== Stage 1/2 ({mac}): Flashing signed Xiaomi 2.1.1_0159c intermediate firmware ===")
        await flash_single_stage(
            mac=mac,
            mi_token=mi_token,
            firmware_path=None,
            sign_file_path=None,
            variant="signed-original",
            force_no_sign=True,
            cache_dir=cache_dir,
            refresh_cache=refresh_cache,
            adapter=adapter,
            retries=retries,
            scan_timeout_s=scan_timeout_s,
            block_delay_s=block_delay_s,
        )
        print(
            f"Waiting {reboot_wait_s:.0f}s for {mac} to apply Stage 1 update and restart BLE advertising "
            "(note: LCD staying blank on B1.5/B1.6 during this intermediate step is normal)..."
        )
        await asyncio.sleep(reboot_wait_s)
        print(f"=== Stage 2/2 ({mac}): Flashing custom pvvx/ATC firmware + 2.1.1_0159c signature ===")
        await flash_single_stage(
            mac=mac,
            mi_token=mi_token,
            firmware_path=firmware_path,
            sign_file_path=sign_file_path,
            variant="custom",
            force_no_sign=force_no_sign,
            cache_dir=cache_dir,
            refresh_cache=False,
            adapter=adapter,
            retries=max(retries, 6),
            scan_timeout_s=scan_timeout_s,
            block_delay_s=block_delay_s,
        )
    else:
        variant = "signed-original" if signed_original else "custom"
        await flash_single_stage(
            mac=mac,
            mi_token=mi_token,
            firmware_path=firmware_path,
            sign_file_path=sign_file_path,
            variant=variant,
            force_no_sign=force_no_sign,
            cache_dir=cache_dir,
            refresh_cache=refresh_cache,
            adapter=adapter,
            retries=retries,
            scan_timeout_s=scan_timeout_s,
            block_delay_s=block_delay_s,
        )


# ---------------------------------------------------------------------------
# Custom Firmware Configuration Helpers & GATT Operations (0x1f10 / 0x1f1f)
# ---------------------------------------------------------------------------

def _parse_on_off(value: str) -> bool:
    v = value.strip().lower()
    if v in ("1", "true", "on", "yes", "enable", "enabled"):
        return True
    if v in ("0", "false", "off", "no", "disable", "disabled"):
        return False
    raise ValueError(f"Invalid boolean '{value}' (expected on/off, true/false, or 1/0).")


def build_dev_name_packet(name: Optional[str]) -> bytes:
    """Build command `0x01` to set custom device name (1..18 UTF-8 bytes) or reset (`None`/`''`)."""
    if not name:
        return b"\x01\x00"
    raw = name.encode("utf-8")
    if not 1 <= len(raw) <= 18:
        raise ValueError("Device name must be 1..18 UTF-8 bytes.")
    return b"\x01" + raw


def build_bindkey_packet(bindkey_hex: str) -> bytes:
    """Build command `0x18` to write a 16-byte (32 hex) beacon bindkey into EEP."""
    cleaned = re.sub(r"[^0-9a-fA-F]", "", bindkey_hex or "")
    if len(cleaned) != 32:
        raise ValueError("BindKey must be exactly 32 hex characters (16 bytes).")
    return b"\x18" + bytes.fromhex(cleaned)


def build_pincode_packet(pin: str) -> bytes:
    """Build command `0x70` to set/disable BLE pincode (`000000` disables)."""
    s = (pin or "").strip()
    if len(s) != 6 or not s.isdigit():
        raise ValueError("PinCode must be 6 decimal digits (000000..999999; 000000 disables).")
    return struct.pack("<BI", 0x70, int(s, 10))


def build_sync_time_packet(local_epoch_s: Optional[int] = None) -> bytes:
    """
    Build command `0x23` matching TelinkMiFlasher.html (`Date.now()/1000 - getTimezoneOffset()*60`),
    sending local wall-clock epoch seconds so the LCD displays local time.
    """
    if local_epoch_s is None:
        now_local = datetime.now().astimezone()
        utc_offset_s = int((now_local.utcoffset().total_seconds()) if now_local.utcoffset() else 0)
        local_epoch_s = int(time.time()) + utc_offset_s
    return struct.pack("<BI", 0x23, int(local_epoch_s) & 0xFFFFFFFF)


def build_comfort_packet(temp_lo: float, temp_hi: float, humi_lo: float, humi_hi: float) -> bytes:
    """Build command `0x20` for comfort temperature/humidity thresholds."""
    return struct.pack(
        "<Bhhhh",
        0x20,
        int(round(temp_lo * 100.0)),
        int(round(temp_hi * 100.0)),
        int(round(humi_lo * 100.0)),
        int(round(humi_hi * 100.0)),
    )


def build_sensor_cal_packet(
    temp_k: float,
    temp_z: float,
    humi_k: float,
    humi_z: float,
) -> bytes:
    """Build command `0x25` for sensor linear calibration (`T = RegT*Tk/65536 + Tz`, firmware >= 4.7)."""
    return struct.pack(
        "<BIIhh",
        0x25,
        int(round(temp_k * 100.0)) & 0xFFFFFFFF,
        int(round(humi_k * 100.0)) & 0xFFFFFFFF,
        int(round(temp_z * 100.0)),
        int(round(humi_z * 100.0)),
    )


def parse_custom_notification(pkt: bytes) -> Dict[str, Any]:
    """Decode common response packets from custom firmware characteristic `0x1f1f`."""
    if not pkt:
        return {"raw": ""}
    blkid = pkt[0]
    payload = pkt[1:]
    if blkid == 0x55 and len(pkt) >= 10:
        return {"id": 0x55, "type": "config", "config": PvvxConfig.from_bytes(pkt)}
    if blkid == 0x01:
        name = bytes(payload).split(b"\x00", 1)[0].decode("utf-8", errors="replace")
        return {"id": 0x01, "type": "name", "name": name}
    if blkid == 0x18:
        if len(payload) >= 16:
            return {"id": 0x18, "type": "bindkey", "bindkey": bytes(payload[:16]).hex()}
        return {"id": 0x18, "type": "bindkey", "bindkey": None}
    if blkid == 0x20 and len(payload) >= 8:
        t_lo, t_hi, h_lo, h_hi = struct.unpack_from("<hhHH", payload, 0)
        return {
            "id": 0x20,
            "type": "comfort",
            "temp_lo": t_lo / 100.0,
            "temp_hi": t_hi / 100.0,
            "humi_lo": h_lo / 100.0,
            "humi_hi": h_hi / 100.0,
        }
    if blkid == 0x23 and len(payload) >= 4:
        cur_t = struct.unpack_from("<I", payload, 0)[0]
        return {"id": 0x23, "type": "time", "dev_time": cur_t}
    if blkid in (0x25, 0x26) and len(payload) >= 12:
        tk, hk, tz, hz = struct.unpack_from("<IIhh", payload, 0)
        return {
            "id": blkid,
            "type": "sensor_cal",
            "temp_k": tk / 100.0,
            "humi_k": hk / 100.0,
            "temp_z": tz / 100.0,
            "humi_z": hz / 100.0,
        }
    return {"id": blkid, "type": "raw", "raw": pkt.hex()}


def apply_pvvx_config_updates(cfg: PvvxConfig, updates: Dict[str, Any]) -> bool:
    """
    Apply high-level configuration fields onto a `PvvxConfig` instance.
    Returns `True` if any `0x55` field was specified.
    """
    changed = False

    preset = updates.get("preset")
    if preset:
        p = str(preset).strip().lower()
        if p == "atb-flow":
            # Optimized for ATB-Flow / Tasmota 16.0 BLE bridge:
            # BTHome v2 unencrypted + AdFlags + 2500 ms adv + 10 s measure interval
            cfg.set_adv_type("bthome2")
            cfg.encrypted = False
            cfg.adv_flags = True
            cfg.set_adv_interval_ms(2500.0)
            cfg.set_measure_interval(4)
            changed = True
        else:
            raise ValueError(f"Unknown config preset '{preset}' (supported: atb-flow).")

    if updates.get("adv_type") is not None:
        cfg.set_adv_type(str(updates["adv_type"]))
        changed = True
    if updates.get("adv_flags") is not None:
        cfg.adv_flags = bool(updates["adv_flags"])
        changed = True
    if updates.get("encrypted") is not None:
        cfg.encrypted = bool(updates["encrypted"])
        changed = True
    if updates.get("bt5_phy") is not None:
        cfg.bt5_phy = bool(updates["bt5_phy"])
        changed = True
    if updates.get("long_range") is not None:
        cfg.long_range = bool(updates["long_range"])
        changed = True
    if updates.get("screen_off") is not None:
        cfg.screen_off = bool(updates["screen_off"])
        changed = True
    if updates.get("temp_unit") is not None:
        cfg.temp_unit = str(updates["temp_unit"])
        changed = True
    if updates.get("comfort") is not None:
        cfg.comfort = bool(updates["comfort"])
        changed = True
    if updates.get("blinking") is not None:
        cfg.blinking = bool(updates["blinking"])
        changed = True
    if updates.get("show_batt") is not None:
        cfg.show_batt = bool(updates["show_batt"])
        changed = True
    if updates.get("smiley") is not None:
        cfg.smiley = int(updates["smiley"])
        changed = True
    if updates.get("tx_measures") is not None:
        cfg.tx_measures = bool(updates["tx_measures"])
        changed = True
    if updates.get("lp_measures") is not None:
        cfg.lp_measures = bool(updates["lp_measures"])
        changed = True
    if updates.get("adv_interval_ms") is not None:
        cfg.set_adv_interval_ms(float(updates["adv_interval_ms"]))
        changed = True
    if updates.get("measure_interval") is not None:
        cfg.set_measure_interval(int(updates["measure_interval"]))
        changed = True
    if updates.get("rf_tx_power") is not None:
        cfg.set_rf_tx_power(int(updates["rf_tx_power"]))
        changed = True
    if updates.get("connect_latency_ms") is not None:
        cfg.set_connect_latency_ms(float(updates["connect_latency_ms"]))
        changed = True
    if updates.get("lcd_refresh_s") is not None:
        cfg.set_lcd_refresh_s(float(updates["lcd_refresh_s"]))
        changed = True
    if updates.get("av_meas_mem") is not None:
        cfg.av_meas_mem = max(0, min(255, int(updates["av_meas_mem"])))
        changed = True
    if updates.get("temp_offset") is not None:
        cfg.set_temp_offset_c(float(updates["temp_offset"]))
        changed = True
    if updates.get("humi_offset") is not None:
        cfg.set_humi_offset_pct(float(updates["humi_offset"]))
        changed = True
    if updates.get("adv_delay_steps") is not None:
        cfg.set_adv_delay_steps(int(updates["adv_delay_steps"]))
        changed = True
    if updates.get("event_adv_cnt") is not None:
        cfg.set_event_adv_cnt(int(updates["event_adv_cnt"]))
        changed = True

    return changed


async def _send_custom_and_wait(
    client: Any,
    cfg_char: Any,
    cmd: bytes,
    expect_ids: Optional[Sequence[int]] = None,
    timeout_s: float = 2.5,
) -> Optional[bytes]:
    """Write a command to `0x1f1f` and await a matching notification if requested."""
    q: asyncio.Queue[bytes] = asyncio.Queue()

    def _on_notify(_sender: Any, data: bytearray) -> None:
        q.put_nowait(bytes(data))

    has_notify = "notify" in (cfg_char.properties or [])
    if has_notify:
        await client.start_notify(cfg_char, _on_notify)
    try:
        await write_gatt_auto(client, cfg_char, cmd)
        if not has_notify:
            return None
        deadline = time.monotonic() + timeout_s
        while True:
            rem = deadline - time.monotonic()
            if rem <= 0:
                return None
            try:
                pkt = await asyncio.wait_for(q.get(), timeout=rem)
            except asyncio.TimeoutError:
                return None
            if expect_ids is None or (pkt and pkt[0] in expect_ids):
                return pkt
    finally:
        if has_notify:
            try:
                await client.stop_notify(cfg_char)
            except Exception:
                pass


async def configure_connected_device(
    client: Any,
    profile: DeviceProfile,
    updates: Dict[str, Any],
    log: Callable[[str], None] = print,
) -> Dict[str, Any]:
    """Read and/or apply pvvx custom firmware configuration on an already-connected client."""
    if not profile.custom_enabled:
        raise FlasherError(
            f"Device {profile.address} ({profile.name}) does not expose custom service 0x1f10 "
            "(flash custom pvvx/ATC firmware first)."
        )
    cfg_char = _get_service_char(client, UUID_SVC_CUSTOM, UUID_CHR_CUSTOM_CFG)
    if cfg_char is None:
        raise FlasherError(f"Custom config characteristic {UUID_CHR_CUSTOM_CFG} not found on {profile.address}.")

    result: Dict[str, Any] = {"address": profile.address, "name": profile.name}

    # Optional factory default reset (0x56)
    if updates.get("reset_default"):
        log("Restoring pvvx default configuration (0x56)...")
        pkt = await _send_custom_and_wait(client, cfg_char, b"\x56", expect_ids=(0x55,), timeout_s=2.5)
        if pkt is None:
            pkt = await _send_custom_and_wait(client, cfg_char, b"\x55", expect_ids=(0x55,), timeout_s=2.5)
        if pkt is not None:
            profile.pvvx_config = PvvxConfig.from_bytes(pkt)

    # Ensure we have the latest 0x55 config block
    if profile.pvvx_config is None:
        pkt = await _send_custom_and_wait(client, cfg_char, b"\x55", expect_ids=(0x55,), timeout_s=2.5)
        if pkt is None:
            raise FlasherError(f"Failed to read 0x55 config from {profile.address}.")
        profile.pvvx_config = PvvxConfig.from_bytes(pkt)

    cfg = profile.pvvx_config
    if apply_pvvx_config_updates(cfg, updates):
        set_pkt = cfg.to_set_packet()
        log(f"Writing custom config (0x55): {set_pkt.hex()}")
        resp = await _send_custom_and_wait(client, cfg_char, set_pkt, expect_ids=(0x55,), timeout_s=2.5)
        if resp is None:
            resp = await _send_custom_and_wait(client, cfg_char, b"\x55", expect_ids=(0x55,), timeout_s=2.5)
        if resp is not None:
            cfg = PvvxConfig.from_bytes(resp)
            profile.pvvx_config = cfg

    result["config"] = cfg.to_dict()

    # Optional secondary commands
    if updates.get("reset_name"):
        pkt = await _send_custom_and_wait(client, cfg_char, build_dev_name_packet(None), expect_ids=(0x01,))
        if pkt is not None:
            result["dev_name"] = parse_custom_notification(pkt).get("name")
    elif updates.get("set_name") is not None:
        pkt = await _send_custom_and_wait(
            client, cfg_char, build_dev_name_packet(str(updates["set_name"])), expect_ids=(0x01,)
        )
        if pkt is not None:
            result["dev_name"] = parse_custom_notification(pkt).get("name")
    elif updates.get("get_name"):
        pkt = await _send_custom_and_wait(client, cfg_char, b"\x01", expect_ids=(0x01,))
        if pkt is not None:
            result["dev_name"] = parse_custom_notification(pkt).get("name")

    if updates.get("bindkey") is not None:
        pkt = await _send_custom_and_wait(
            client, cfg_char, build_bindkey_packet(str(updates["bindkey"])), expect_ids=(0x18,)
        )
        if pkt is not None:
            result["bindkey"] = parse_custom_notification(pkt).get("bindkey")
    elif updates.get("get_bindkey"):
        pkt = await _send_custom_and_wait(client, cfg_char, b"\x18", expect_ids=(0x18,))
        if pkt is not None:
            result["bindkey"] = parse_custom_notification(pkt).get("bindkey")

    if updates.get("pincode") is not None:
        await _send_custom_and_wait(
            client, cfg_char, build_pincode_packet(str(updates["pincode"])), expect_ids=None, timeout_s=0.6
        )
        result["pincode_set"] = True

    if updates.get("sync_time"):
        pkt = await _send_custom_and_wait(client, cfg_char, build_sync_time_packet(), expect_ids=(0x23,))
        result["time_synced"] = True
        if pkt is not None:
            result["dev_time"] = parse_custom_notification(pkt).get("dev_time")

    if updates.get("comfort_range") is not None:
        t_lo, t_hi, h_lo, h_hi = updates["comfort_range"]
        pkt = await _send_custom_and_wait(
            client, cfg_char, build_comfort_packet(t_lo, t_hi, h_lo, h_hi), expect_ids=(0x20,)
        )
        if pkt is not None:
            result["comfort_range"] = parse_custom_notification(pkt)

    if updates.get("reset_sensor_cal"):
        pkt = await _send_custom_and_wait(client, cfg_char, b"\x26", expect_ids=(0x25, 0x26))
        if pkt is not None:
            result["sensor_cal"] = parse_custom_notification(pkt)
    elif updates.get("sensor_cal") is not None:
        tk, tz, hk, hz = updates["sensor_cal"]
        pkt = await _send_custom_and_wait(
            client, cfg_char, build_sensor_cal_packet(tk, tz, hk, hz), expect_ids=(0x25,)
        )
        if pkt is not None:
            result["sensor_cal"] = parse_custom_notification(pkt)
    elif updates.get("get_sensor_cal"):
        pkt = await _send_custom_and_wait(client, cfg_char, b"\x25", expect_ids=(0x25,))
        if pkt is not None:
            result["sensor_cal"] = parse_custom_notification(pkt)

    if updates.get("raw_cmd"):
        raw_hex = re.sub(r"[^0-9a-fA-F]", "", str(updates["raw_cmd"]))
        if not raw_hex or len(raw_hex) % 2 != 0:
            raise ValueError("--raw-cmd must be an even-length hex string (e.g. '55' or '33ff').")
        raw_bytes = bytes.fromhex(raw_hex)
        pkt = await _send_custom_and_wait(client, cfg_char, raw_bytes, expect_ids=None, timeout_s=2.0)
        result["raw_cmd_tx"] = raw_hex
        result["raw_cmd_rx"] = pkt.hex() if pkt is not None else None

    return result


async def run_config(
    mac: str,
    updates: Dict[str, Any],
    json_output: bool = False,
    adapter: Optional[str] = None,
    retries: int = 5,
    scan_timeout_s: float = 12.0,
) -> Dict[str, Any]:
    client, adv_name = await connect_with_retries(
        mac=mac, adapter=adapter, scan_timeout_s=scan_timeout_s, retries=retries
    )
    try:
        profile = await probe_device_profile(client, normalize_mac(mac), adv_name)
        res = await configure_connected_device(
            client=client,
            profile=profile,
            updates=updates,
            log=(lambda _s: None) if json_output else print,
        )
        if not json_output:
            print(f"Device:            {profile.address} ({profile.name}, custom v{profile.pvvx_config.ver_str})")
            print(profile.pvvx_config.format_summary())
            if "dev_name" in res:
                print(f"  Device Name:     {res['dev_name']}")
            if "bindkey" in res:
                print(f"  EEP BindKey:     {res['bindkey'] or '<none>'}")
            if "sensor_cal" in res:
                sc = res["sensor_cal"]
                print(
                    f"  Sensor Cal:      Tk={sc.get('temp_k')}, Tz={sc.get('temp_z')}, "
                    f"Hk={sc.get('humi_k')}, Hz={sc.get('humi_z')}"
                )
            if "raw_cmd_rx" in res:
                print(f"  Raw Cmd RX:      {res['raw_cmd_rx'] or '<none>'}")
        return res
    finally:
        if getattr(client, "is_connected", False):
            try:
                await client.disconnect()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# CLI Argument Parsing & Entrypoint
# ---------------------------------------------------------------------------

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="telink_mi_flasher.py",
        description=(
            "Linux CLI port of pvvx TelinkMiFlasher.html with full support for "
            "Xiaomi LYWSD03MMC firmware 2.1.1_0159 OTA unlocking & custom firmware flashing."
        ),
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # --- scan ---
    p_scan = sub.add_parser("scan", help="Scan nearby BLE thermometers (LYWSD03MMC / ATC_* / PVVX)")
    p_scan.add_argument("--timeout", type=float, default=8.0, help="BLE scan duration in seconds (default: 8)")
    p_scan.add_argument("--adapter", help="Linux BlueZ adapter name, e.g. hci0")
    p_scan.add_argument(
        "--filter",
        default="LYWSD03MMC,ATC_,MJWSD,MHO-",
        help="Comma-separated BLE name prefixes to show (pass '' to show all BLE devices)",
    )
    p_scan.add_argument(
        "--tokens-file",
        type=Path,
        help="Optional Xiaomi-cloud-tokens-extractor JSON/text file to highlight matched devices",
    )

    # --- list-tokens ---
    p_tokens = sub.add_parser(
        "list-tokens",
        help="List BLE credentials parsed from a Xiaomi-cloud-tokens-extractor JSON or text file",
    )
    p_tokens.add_argument("tokens_file", type=Path, help="Path to tokens JSON or saved console output")
    p_tokens.add_argument(
        "--only-lywsd03mmc",
        action="store_true",
        help="Show only LYWSD03MMC (miaomiaoce.sensor_ht.t2) entries",
    )

    # --- info ---
    p_info = sub.add_parser("info", help="Connect to a sensor and display firmware/hardware/OTA status")
    p_info.add_argument("--mac", required=True, help="Target BLE MAC address (AA:BB:CC:DD:EE:FF)")
    p_info.add_argument("--adapter", help="Linux BlueZ adapter name, e.g. hci0")
    p_info.add_argument("--retries", type=int, default=5, help="Connection retries (default: 5)")
    p_info.add_argument("--scan-timeout", type=float, default=12.0, help="Per-attempt scan timeout in seconds")

    # --- flash ---
    p_flash = sub.add_parser(
        "flash",
        help="Authenticate (Mi Token) and flash custom pvvx/ATC firmware via BLE OTA",
    )
    target_grp = p_flash.add_mutually_exclusive_group(required=True)
    target_grp.add_argument("--mac", help="Target BLE MAC address (AA:BB:CC:DD:EE:FF)")
    target_grp.add_argument(
        "--all-lywsd03mmc",
        action="store_true",
        help="Sequentially flash all LYWSD03MMC sensors listed in --tokens-file",
    )
    p_flash.add_argument("--token", help="24-hex-character (12-byte) Mi Token")
    p_flash.add_argument(
        "--bind-key",
        help="Optional 32-hex-character (16-byte) Mi Bind Key (validated to catch swapped keys)",
    )
    p_flash.add_argument("--known-id", help="Optional Xiaomi Device ID (e.g. blt.3.xxx)")
    p_flash.add_argument(
        "--tokens-file",
        type=Path,
        help="Xiaomi-cloud-tokens-extractor output file (JSON or text) to auto-lookup --token by --mac",
    )
    p_flash.add_argument(
        "--firmware",
        type=Path,
        help="Local Telink/Zigbee OTA binary (.bin/.zigbee). If omitted, downloads latest ATC_vNN.bin from pvvx",
    )
    p_flash.add_argument(
        "--sign-file",
        type=Path,
        help=f"Local signature file for 2.1.1_0159. If omitted, auto-downloads {SIGN_FILE_211_0159} when needed",
    )
    mode_grp = p_flash.add_mutually_exclusive_group()
    mode_grp.add_argument(
        "--signed-original",
        action="store_true",
        help="Flash Xiaomi-signed Original_OTA_Xiaomi_LYWSD03MMC_v2.1.1_0159c.bin (Stage 1 for 2025 B1.5/B1.6)",
    )
    mode_grp.add_argument(
        "--two-stage",
        action="store_true",
        help=(
            "Run Stage 1 (signed Original 2.1.1_0159c), wait for reboot, then Stage 2 "
            "(custom ATC_vNN.bin + sign_2.1.1_0159c.bin) for 2025+ B1.5/B1.6/B1.1 sensors"
        ),
    )
    p_flash.add_argument(
        "--no-sign",
        action="store_true",
        help="Never append sign_2.1.1_0159c.bin even if 2.1.1_0159 is detected",
    )
    p_flash.add_argument(
        "--reboot-wait",
        type=float,
        default=18.0,
        help="Seconds to wait between Stage 1 and Stage 2 in --two-stage mode (default: 18)",
    )
    p_flash.add_argument(
        "--block-delay-ms",
        type=float,
        default=2.0,
        help="Inter-packet delay in ms for WriteWithoutResponse blocks (default: 2.0)",
    )
    p_flash.add_argument("--cache-dir", type=Path, default=default_cache_dir(), help="Cache folder for upstream files")
    p_flash.add_argument("--refresh-cache", action="store_true", help="Re-download cached upstream files")
    p_flash.add_argument("--adapter", help="Linux BlueZ adapter name, e.g. hci0")
    p_flash.add_argument("--retries", type=int, default=5, help="Connection retries (default: 5)")
    p_flash.add_argument("--scan-timeout", type=float, default=12.0, help="Per-attempt scan timeout in seconds")
    p_flash.add_argument(
        "--config-preset",
        choices=["atb-flow"],
        help="Optional pvvx config preset to apply after flashing custom firmware (reconnects after reboot)",
    )

    # --- config ---
    p_cfg = sub.add_parser(
        "config",
        help="Read and/or update custom pvvx/ATC firmware configuration (service 0x1f10 / 0x1f1f)",
    )
    cfg_target = p_cfg.add_mutually_exclusive_group(required=True)
    cfg_target.add_argument("--mac", help="Target BLE MAC address (AA:BB:CC:DD:EE:FF)")
    cfg_target.add_argument(
        "--macs",
        help="Comma-separated list of BLE MAC addresses to configure sequentially",
    )
    cfg_target.add_argument(
        "--all-lywsd03mmc",
        action="store_true",
        help="Sequentially configure all LYWSD03MMC sensors listed in --tokens-file",
    )
    p_cfg.add_argument(
        "--tokens-file",
        type=Path,
        help="Xiaomi-cloud-tokens-extractor file (required with --all-lywsd03mmc)",
    )
    p_cfg.add_argument("--json", action="store_true", help="Output configuration as JSON")
    p_cfg.add_argument(
        "--preset",
        choices=["atb-flow"],
        help="Apply a predefined config preset ('atb-flow' = BTHome v2, unencrypted, AdFlags on, 2500ms/10s)",
    )
    p_cfg.add_argument(
        "--reset-default",
        action="store_true",
        help="Send command 0x56 to restore firmware default configuration before applying other flags",
    )
    p_cfg.add_argument(
        "--adv-type",
        choices=["atc1441", "pvvx", "mijia", "bthome2", "bthome1", "all"],
        help="Advertising packet format (recommended for Tasmota 16.0+: bthome2)",
    )
    p_cfg.add_argument("--adv-flags", type=_parse_on_off, metavar="{on,off}", help="Enable/disable AdFlags (flg2 bit 4)")
    p_cfg.add_argument("--encrypted", type=_parse_on_off, metavar="{on,off}", help="Enable/disable encrypted beacons")
    p_cfg.add_argument("--bt5-phy", type=_parse_on_off, metavar="{on,off}", help="Enable/disable BT5+ PHY / CSA2")
    p_cfg.add_argument("--long-range", type=_parse_on_off, metavar="{on,off}", help="Enable/disable Coded PHY S8 long range")
    p_cfg.add_argument("--screen-off", type=_parse_on_off, metavar="{on,off}", help="Turn LCD off (on/off)")
    p_cfg.add_argument("--temp-unit", choices=["C", "F", "c", "f"], help="Display temperature unit (C or F)")
    p_cfg.add_argument("--comfort", type=_parse_on_off, metavar="{on,off}", help="Show comfort smiley indicator (on/off)")
    p_cfg.add_argument("--blinking", type=_parse_on_off, metavar="{on,off}", help="Show blinking/clock mode on LCD (on/off)")
    p_cfg.add_argument("--show-batt", type=_parse_on_off, metavar="{on,off}", help="Show battery indicator on LCD (on/off)")
    p_cfg.add_argument("--smiley", type=int, choices=range(0, 8), help="Fixed smiley / screen mode (0..7)")
    p_cfg.add_argument("--tx-measures", type=_parse_on_off, metavar="{on,off}", help="Transmit measurements when connected")
    p_cfg.add_argument("--lp-measures", type=_parse_on_off, metavar="{on,off}", help="Low-power measurement mode (v4.8+)")
    p_cfg.add_argument("--adv-interval-ms", type=float, help="Advertising interval in ms (62.5..15937.5, step 62.5)")
    p_cfg.add_argument("--measure-interval", type=int, help="Measure interval multiplier (1..255 x adv-interval)")
    p_cfg.add_argument("--rf-tx-power", type=int, help="RF TX power register (VBAT: 23..63, VANT: 130..191)")
    p_cfg.add_argument("--connect-latency-ms", type=float, help="BLE connection latency in ms (20..1000, step 20)")
    p_cfg.add_argument("--lcd-refresh-s", type=float, help="Minimum LCD refresh interval in s (0.5..12.75, step 0.05)")
    p_cfg.add_argument("--av-meas-mem", type=int, help="Flash averaging memory count (0=off..255)")
    p_cfg.add_argument("--temp-offset", type=float, help="Temperature offset in °C (-12.7..12.7, firmware < 4.7)")
    p_cfg.add_argument("--humi-offset", type=float, help="Humidity offset in %% (-12.7..12.7, firmware < 4.7)")
    p_cfg.add_argument("--adv-delay-steps", type=int, help="Pseudo-random adv delay in 0.625 ms steps (0..15, fw > 5.0)")
    p_cfg.add_argument("--event-adv-cnt", type=int, help="Event beacon duplicate count (5..255, fw > 5.0)")
    p_cfg.add_argument("--name", dest="set_name", help="Set custom BLE device name (1..18 chars)")
    p_cfg.add_argument("--reset-name", action="store_true", help="Reset BLE device name to default (ATC_xxxx)")
    p_cfg.add_argument("--get-name", action="store_true", help="Query current BLE device name (0x01)")
    p_cfg.add_argument("--bindkey", help="Set 32-hex EEP beacon bindkey (0x18)")
    p_cfg.add_argument("--get-bindkey", action="store_true", help="Read current EEP beacon bindkey (0x18)")
    p_cfg.add_argument("--pincode", help="Set 6-digit BLE pincode (000000 disables)")
    p_cfg.add_argument("--sync-time", action="store_true", help="Sync device clock to host local time (0x23)")
    p_cfg.add_argument(
        "--comfort-range",
        help="Set comfort thresholds as t_lo,t_hi,h_lo,h_hi (e.g. 20.0,25.0,35.0,60.0)",
    )
    p_cfg.add_argument(
        "--sensor-cal",
        help="Set linear sensor calibration Tk,Tz,Hk,Hz (e.g. 100.0,0.0,100.0,0.0 for fw >= 4.7)",
    )
    p_cfg.add_argument("--get-sensor-cal", action="store_true", help="Query linear sensor calibration (0x25)")
    p_cfg.add_argument("--reset-sensor-cal", action="store_true", help="Reset linear sensor calibration (0x26)")
    p_cfg.add_argument("--raw-cmd", help="Send raw hex command to custom config characteristic 0x1f1f")
    p_cfg.add_argument("--adapter", help="Linux BlueZ adapter name, e.g. hci0")
    p_cfg.add_argument("--retries", type=int, default=5, help="Connection retries (default: 5)")
    p_cfg.add_argument("--scan-timeout", type=float, default=12.0, help="Per-attempt scan timeout in seconds")

    # --- build-image ---
    p_build = sub.add_parser(
        "build-image",
        help="Validate a firmware file offline and optionally build a 2.1.1_0159-signed OTA image",
    )
    p_build.add_argument("--firmware", type=Path, help="Input firmware .bin (if omitted, downloads latest ATC_vNN.bin)")
    p_build.add_argument("--sign-file", type=Path, help=f"Local {SIGN_FILE_211_0159} (downloads if --sign-211-0159 set)")
    p_build.add_argument(
        "--sign-211-0159",
        action="store_true",
        help=f"Append {SIGN_FILE_211_0159} exploit signature block",
    )
    p_build.add_argument("--output", type=Path, help="Write prepared 16-byte-aligned OTA payload to this file")
    p_build.add_argument("--cache-dir", type=Path, default=default_cache_dir(), help="Cache folder for upstream files")

    return parser


def _parse_float_quad(spec: str, flag_name: str) -> Tuple[float, float, float, float]:
    parts = [p.strip() for p in (spec or "").split(",") if p.strip()]
    if len(parts) != 4:
        raise ValueError(f"{flag_name} expects 4 comma-separated numbers, got '{spec}'.")
    return float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])


def _extract_config_updates(args: argparse.Namespace) -> Dict[str, Any]:
    updates: Dict[str, Any] = {
        "preset": args.preset,
        "reset_default": args.reset_default,
        "adv_type": args.adv_type,
        "adv_flags": args.adv_flags,
        "encrypted": args.encrypted,
        "bt5_phy": args.bt5_phy,
        "long_range": args.long_range,
        "screen_off": args.screen_off,
        "temp_unit": args.temp_unit,
        "comfort": args.comfort,
        "blinking": args.blinking,
        "show_batt": args.show_batt,
        "smiley": args.smiley,
        "tx_measures": args.tx_measures,
        "lp_measures": args.lp_measures,
        "adv_interval_ms": args.adv_interval_ms,
        "measure_interval": args.measure_interval,
        "rf_tx_power": args.rf_tx_power,
        "connect_latency_ms": args.connect_latency_ms,
        "lcd_refresh_s": args.lcd_refresh_s,
        "av_meas_mem": args.av_meas_mem,
        "temp_offset": args.temp_offset,
        "humi_offset": args.humi_offset,
        "adv_delay_steps": args.adv_delay_steps,
        "event_adv_cnt": args.event_adv_cnt,
        "set_name": args.set_name,
        "reset_name": args.reset_name,
        "get_name": args.get_name,
        "bindkey": args.bindkey,
        "get_bindkey": args.get_bindkey,
        "pincode": args.pincode,
        "sync_time": args.sync_time,
        "get_sensor_cal": args.get_sensor_cal,
        "reset_sensor_cal": args.reset_sensor_cal,
        "raw_cmd": args.raw_cmd,
    }
    if args.comfort_range is not None:
        updates["comfort_range"] = _parse_float_quad(args.comfort_range, "--comfort-range")
    if args.sensor_cal is not None:
        updates["sensor_cal"] = _parse_float_quad(args.sensor_cal, "--sensor-cal")
    return updates


async def _run_flash_and_optional_config(
    *,
    mac: str,
    mi_token: Optional[str],
    firmware_path: Optional[Path],
    sign_file_path: Optional[Path],
    signed_original: bool,
    two_stage: bool,
    force_no_sign: bool,
    reboot_wait_s: float,
    cache_dir: Path,
    refresh_cache: bool,
    adapter: Optional[str],
    retries: int,
    scan_timeout_s: float,
    block_delay_s: float,
    config_preset: Optional[str],
) -> None:
    await run_flash_workflow(
        mac=mac,
        mi_token=mi_token,
        firmware_path=firmware_path,
        sign_file_path=sign_file_path,
        signed_original=signed_original,
        two_stage=two_stage,
        force_no_sign=force_no_sign,
        reboot_wait_s=reboot_wait_s,
        cache_dir=cache_dir,
        refresh_cache=refresh_cache,
        adapter=adapter,
        retries=retries,
        scan_timeout_s=scan_timeout_s,
        block_delay_s=block_delay_s,
    )
    if config_preset and not signed_original:
        print(f"Waiting {reboot_wait_s:.0f}s for {mac} to reboot before applying preset '{config_preset}'...")
        await asyncio.sleep(reboot_wait_s)
        await run_config(
            mac=mac,
            updates={"preset": config_preset},
            json_output=False,
            adapter=adapter,
            retries=max(retries, 6),
            scan_timeout_s=scan_timeout_s,
        )


def _cmd_list_tokens(args: argparse.Namespace) -> int:
    creds = parse_tokens_file(args.tokens_file)
    if args.only_lywsd03mmc:
        creds = [
            c
            for c in creds
            if (c.model or "") == "miaomiaoce.sensor_ht.t2"
            or "LYWSD03MMC" in (c.name or "").upper()
        ]
    if not creds:
        print(f"No matching BLE device credentials found in '{args.tokens_file}'.")
        return 1

    print(f"{'MAC Address':<19} {'Token (24 hex)':<26} {'Bind Key (32 hex)':<34} {'Model / Name'}")
    print("-" * 105)
    for c in creds:
        label = " / ".join(x for x in (c.model, c.name) if x)
        print(
            f"{(c.mac or 'N/A'):<19} {(c.token or 'N/A'):<26} "
            f"{(c.bind_key or 'N/A'):<34} {label}"
        )
    return 0


def _cmd_build_image(args: argparse.Namespace) -> int:
    if args.firmware is not None:
        raw_fw = args.firmware.read_bytes()
        fw_label = str(args.firmware)
    else:
        fw_index = json.loads(fetch_repo_asset("firmware.json", cache_dir=args.cache_dir).decode("utf-8"))
        rel_fw = resolve_upstream_firmware_path(fw_index, 0, variant="custom")
        raw_fw = fetch_repo_asset(rel_fw, cache_dir=args.cache_dir)
        fw_label = rel_fw

    sign_bytes: Optional[bytes] = None
    if args.sign_file is not None:
        sign_bytes = args.sign_file.read_bytes()
    elif args.sign_211_0159:
        sign_bytes = fetch_repo_asset(f"signfiles/{SIGN_FILE_211_0159}", cache_dir=args.cache_dir)

    ota_image, info, sig_applied = prepare_ota_image(raw_firmware=raw_fw, sign_data=sign_bytes)
    print(f"Source firmware:  {fw_label}")
    print(f"Header size:      {info.header_size} bytes (0x{info.header_size:x})")
    print(f"Telink CRC32:     0x{info.file_crc:08x} (valid)")
    print(f"Signature added:  {'yes (2.1.1_0159c)' if sig_applied else ('already MITFOTA-signed' if info.already_signed else 'no')}")
    print(f"Final OTA image:  {len(ota_image)} bytes ({len(ota_image) // 16} blocks)")
    if args.output:
        args.output.write_bytes(ota_image)
        print(f"Wrote OTA image:  {args.output}")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    try:
        if args.command == "list-tokens":
            return _cmd_list_tokens(args)

        if args.command == "build-image":
            return _cmd_build_image(args)

        if args.command == "scan":
            creds = parse_tokens_file(args.tokens_file) if args.tokens_file else None
            return asyncio.run(
                run_scan(
                    timeout_s=args.timeout,
                    adapter=args.adapter,
                    name_filter=args.filter,
                    known_creds=creds,
                )
            )

        if args.command == "info":
            return asyncio.run(
                run_info(
                    mac=args.mac,
                    adapter=args.adapter,
                    retries=args.retries,
                    scan_timeout_s=args.scan_timeout,
                )
            )

        if args.command == "config":
            updates = _extract_config_updates(args)
            if args.mac:
                targets = [normalize_mac(args.mac)]
            elif args.macs:
                targets = [normalize_mac(m.strip()) for m in args.macs.split(",") if m.strip()]
                if not targets:
                    raise FlasherError("No valid MAC addresses provided to --macs.")
            else:
                if not args.tokens_file:
                    raise FlasherError("--all-lywsd03mmc requires --tokens-file <path>.")
                creds_list = parse_tokens_file(args.tokens_file)
                targets = [
                    normalize_mac(c.mac)
                    for c in creds_list
                    if c.mac
                    and (
                        (c.model or "") == "miaomiaoce.sensor_ht.t2"
                        or "LYWSD03MMC" in (c.name or "").upper()
                    )
                ]
                if not targets:
                    raise FlasherError(f"No LYWSD03MMC MAC addresses found in '{args.tokens_file}'.")

            results: List[Dict[str, Any]] = []
            failures: List[Tuple[str, str]] = []
            for idx, target_mac in enumerate(targets, 1):
                if len(targets) > 1 and not args.json:
                    print(f"\n[{idx}/{len(targets)}] Configuring {target_mac}...")
                try:
                    res = asyncio.run(
                        run_config(
                            mac=target_mac,
                            updates=updates,
                            json_output=args.json,
                            adapter=args.adapter,
                            retries=args.retries,
                            scan_timeout_s=args.scan_timeout,
                        )
                    )
                    results.append(res)
                except Exception as exc:
                    if len(targets) == 1:
                        raise
                    print(f"ERROR configuring {target_mac}: {exc}", file=sys.stderr)
                    failures.append((target_mac, str(exc)))

            if args.json:
                print(json.dumps(results[0] if len(targets) == 1 and results else results, indent=2))
            if failures:
                print(f"\nCompleted with {len(failures)}/{len(targets)} failure(s):", file=sys.stderr)
                for fmac, ferr in failures:
                    print(f"  - {fmac}: {ferr}", file=sys.stderr)
                return 1
            return 0

        if args.command == "flash":
            creds_list = parse_tokens_file(args.tokens_file) if args.tokens_file else []

            if args.all_lywsd03mmc:
                if not args.tokens_file:
                    raise FlasherError("--all-lywsd03mmc requires --tokens-file <path>.")
                targets = [
                    c
                    for c in creds_list
                    if c.mac
                    and c.token
                    and (
                        (c.model or "") == "miaomiaoce.sensor_ht.t2"
                        or "LYWSD03MMC" in (c.name or "").upper()
                    )
                ]
                if not targets:
                    raise FlasherError(f"No LYWSD03MMC entries with MAC + Token found in '{args.tokens_file}'.")

                failures: List[Tuple[str, str]] = []
                for idx, item in enumerate(targets, 1):
                    assert item.mac is not None
                    tk, _ = validate_mi_credentials(item.token, item.bind_key)
                    print(f"\n[{idx}/{len(targets)}] Flashing {item.mac} ({item.name or item.model or ''})...")
                    try:
                        asyncio.run(
                            _run_flash_and_optional_config(
                                mac=item.mac,
                                mi_token=tk,
                                firmware_path=args.firmware,
                                sign_file_path=args.sign_file,
                                signed_original=args.signed_original,
                                two_stage=args.two_stage,
                                force_no_sign=args.no_sign,
                                reboot_wait_s=args.reboot_wait,
                                cache_dir=args.cache_dir,
                                refresh_cache=args.refresh_cache and idx == 1,
                                adapter=args.adapter,
                                retries=args.retries,
                                scan_timeout_s=args.scan_timeout,
                                block_delay_s=max(args.block_delay_ms, 0.0) / 1000.0,
                                config_preset=args.config_preset,
                            )
                        )
                    except Exception as exc:
                        print(f"ERROR flashing {item.mac}: {exc}", file=sys.stderr)
                        failures.append((item.mac, str(exc)))

                if failures:
                    print(f"\nCompleted with {len(failures)}/{len(targets)} failure(s):", file=sys.stderr)
                    for fmac, ferr in failures:
                        print(f"  - {fmac}: {ferr}", file=sys.stderr)
                    return 1
                print(f"\nSuccessfully flashed all {len(targets)} LYWSD03MMC device(s)!")
                return 0

            # Single device (--mac)
            target_mac = normalize_mac(args.mac)
            token = args.token
            bind_key = args.bind_key
            if not token and creds_list:
                matched = lookup_credentials_for_mac(creds_list, target_mac)
                if matched:
                    token = matched.token
                    if not bind_key:
                        bind_key = matched.bind_key
                    print(
                        f"Loaded Mi Token for {target_mac} from '{args.tokens_file}'"
                        + (f" ({matched.name})" if matched.name else "")
                    )
                else:
                    print(
                        f"Warning: {target_mac} not found in '{args.tokens_file}'; proceeding without token "
                        "(will only work if sensor already runs custom firmware).",
                        file=sys.stderr,
                    )

            if token:
                token, _ = validate_mi_credentials(token, bind_key)

            asyncio.run(
                _run_flash_and_optional_config(
                    mac=target_mac,
                    mi_token=token,
                    firmware_path=args.firmware,
                    sign_file_path=args.sign_file,
                    signed_original=args.signed_original,
                    two_stage=args.two_stage,
                    force_no_sign=args.no_sign,
                    reboot_wait_s=args.reboot_wait,
                    cache_dir=args.cache_dir,
                    refresh_cache=args.refresh_cache,
                    adapter=args.adapter,
                    retries=args.retries,
                    scan_timeout_s=args.scan_timeout,
                    block_delay_s=max(args.block_delay_ms, 0.0) / 1000.0,
                    config_preset=args.config_preset,
                )
            )
            return 0

    except (FlasherError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)
        return 130

    return 0


if __name__ == "__main__":
    sys.exit(main())
