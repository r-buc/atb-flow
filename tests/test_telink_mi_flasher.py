import asyncio
import hashlib
import hmac
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

# Allow importing tools/telink_mi_flasher.py from repository root
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "tools"))

import telink_mi_flasher as tmf  # noqa: E402


def _make_valid_telink_bin(body_blocks: int = 128) -> bytes:
    """
    Construct a valid minimal Telink OTA firmware image whose `hsize & 0x0f == 4`
    and whose trailing 4 bytes carry `telink_crc32(payload[:hsize-4])`.
    """
    # Total hsize must satisfy (hsize & 0x0F) == 4 and >= 1024.
    hsize = body_blocks * 16 + 4
    buf = bytearray(b"\xaa" * (hsize - 4))
    struct.pack_into("<I", buf, 0x08, tmf.TELINK_MAGIC)
    struct.pack_into("<I", buf, 0x18, hsize)
    crc = tmf.telink_crc32(bytes(buf))
    buf.extend(struct.pack("<I", crc))
    return bytes(buf)


def _make_synthetic_sign_file(
    cl: int = 80,
    fill_len: int = 64,
    patches: Sequence[Tuple[int, int]] = ((0x10, 0x00012313), (0x20, 0x000124A7)),
) -> bytes:
    """
    Build a synthetic signature file encoded identically to `sign_2.1.1_0159c.bin`
    so `append_signature_211_0159()` can be verified deterministically offline.
    """
    head = bytearray(b"\x5a" * cl)
    head[0] = cl & 0xFF
    head[1] = (cl >> 8) & 0xFF
    head[2:4] = b"\x00\x00"
    head[4:12] = tmf.MITFOTA_HEADER[4:12]  # "MI\xefTFOTA"

    crc = tmf.telink_crc32(bytes(head))
    tail = bytearray()

    # Encode fill_len: (crc ^ x) & 0xFFFF == fill_len -> x = (crc ^ fill_len) & 0xFFFF
    x_fill = (crc ^ fill_len) & 0xFFFF
    tail.extend(struct.pack("<H", x_fill))
    crc = (crc & 0xFFFF0000) | x_fill

    for addr, val32 in patches:
        x_addr = (crc ^ addr) & 0xFFFF
        tail.extend(struct.pack("<H", x_addr))
        crc = (crc & 0xFFFF0000) | x_addr

        # Decoder computes: decoded_val = crc ^ x_val, then sets crc = x_val
        x_val = (crc ^ val32) & 0xFFFFFFFF
        tail.extend(struct.pack("<I", x_val))
        crc = x_val

    return bytes(head + tail)


class _FakeChar:
    def __init__(self, uuid: str, properties: Tuple[str, ...]) -> None:
        self.uuid = uuid
        self.properties = list(properties)


class _FakeService:
    def __init__(self, uuid: str, chars: Sequence[_FakeChar]) -> None:
        self.uuid = uuid
        self._chars = {c.uuid.lower(): c for c in chars}

    def get_characteristic(self, uuid: str) -> Optional[_FakeChar]:
        return self._chars.get(uuid.lower())


class _FakeServiceCollection:
    def __init__(self, services: Sequence[_FakeService]) -> None:
        self._services = list(services)
        self._by_uuid = {s.uuid.lower(): s for s in services}

    def __iter__(self):
        return iter(self._services)

    def get_service(self, uuid: str) -> Optional[_FakeService]:
        return self._by_uuid.get(uuid.lower())


class _FakeMi211DeviceClient:
    """
    Simulates a stock Xiaomi LYWSD03MMC running firmware `2.1.1_0159` over BLE:
      - Exposes Device Information (0x180a), Mi Main (ebe0ccb0), Mi Auth (0xfe95),
        and Telink OTA (00010203-...-1912)
      - Executes the MiBLE v2 login protocol (`00000401` session frame + states 1->12->13->14)
      - Verifies host HMAC response using the genuine Mi Token before emitting `21000000`
      - Tracks every Telink OTA write & 8-block status read
    """

    def __init__(
        self,
        mi_token_hex: str,
        dev_random: bytes = b"\xaa" * 16,
        hw_rev: str = "B1.4",
        fw_rev: str = "2.1.1_0159",
        ota_error_at_read: Optional[int] = None,
    ) -> None:
        self.mi_token_hex = mi_token_hex
        self.dev_random = dev_random
        self.is_connected = True
        self.ota_error_at_read = ota_error_at_read
        self._read_count = 0

        self.c_ota = _FakeChar(tmf.UUID_CHR_TELINK_OTA, ("read", "write-without-response"))
        self.c_speed = _FakeChar(tmf.UUID_CHR_MI_SPEED, ("write-without-response",))
        self.c_temp = _FakeChar(tmf.UUID_CHR_MI_TEMP, ("notify",))
        self.c_enc10 = _FakeChar(tmf.UUID_CHR_MI_ENC_10, ("write-without-response", "notify"))
        self.c_enc19 = _FakeChar(tmf.UUID_CHR_MI_ENC_19, ("write-without-response", "notify"))
        self.c_fw = _FakeChar(tmf.UUID_CHR_FW_REV, ("read",))
        self.c_hw = _FakeChar(tmf.UUID_CHR_HW_REV, ("read",))
        self.c_sw = _FakeChar(tmf.UUID_CHR_SW_REV, ("read",))

        self.services = _FakeServiceCollection(
            [
                _FakeService(tmf.UUID_SVC_TELINK_OTA, [self.c_ota]),
                _FakeService(tmf.UUID_SVC_MI_MAIN, [self.c_speed, self.c_temp]),
                _FakeService(tmf.UUID_SVC_MI_AUTH, [self.c_enc10, self.c_enc19]),
                _FakeService(tmf.UUID_SVC_DEV_INFO, [self.c_fw, self.c_hw, self.c_sw]),
            ]
        )

        self._strings = {
            tmf.UUID_CHR_FW_REV: fw_rev.encode("utf-8") + b"\x00",
            tmf.UUID_CHR_HW_REV: hw_rev.encode("utf-8"),
            tmf.UUID_CHR_SW_REV: b"0159",
        }
        self._notify_cbs: Dict[str, Callable[[Any, bytearray], None]] = {}
        self._negotiated = False
        self._awaiting_host_rand = False
        self._awaiting_host_hmac = False
        self._expected_host_hmac = ""
        self._dev_hmac = ""

        self.ota_writes: List[Tuple[bytes, bool]] = []
        self.speed_writes: List[bytes] = []

    async def start_notify(self, char: _FakeChar, cb: Callable[[Any, bytearray], None]) -> None:
        self._notify_cbs[char.uuid.lower()] = cb
        if char.uuid.lower() == tmf.UUID_CHR_MI_TEMP:
            # 22.45 °C, 48 %, 3015 mV
            pkt = struct.pack("<hBH", 2245, 48, 3015)
            asyncio.get_running_loop().call_soon(cb, char, bytearray(pkt))

    async def stop_notify(self, char: _FakeChar) -> None:
        self._notify_cbs.pop(char.uuid.lower(), None)

    async def read_gatt_char(self, char: _FakeChar) -> bytes:
        u = char.uuid.lower()
        if u in self._strings:
            return self._strings[u]
        if u == tmf.UUID_CHR_TELINK_OTA:
            self._read_count += 1
            if self.ota_error_at_read is not None and self._read_count >= self.ota_error_at_read:
                return bytes([2])  # CRC error in data
            return b"\x00"
        return b""

    async def write_gatt_char(self, char: _FakeChar, data: bytes, response: bool = False) -> None:
        u = char.uuid.lower()
        hx = bytes(data).hex()

        if u == tmf.UUID_CHR_TELINK_OTA:
            self.ota_writes.append((bytes(data), response))
            return
        if u == tmf.UUID_CHR_MI_SPEED:
            self.speed_writes.append(bytes(data))
            return

        loop = asyncio.get_running_loop()
        cb19 = self._notify_cbs.get(tmf.UUID_CHR_MI_ENC_19)
        cb10 = self._notify_cbs.get(tmf.UUID_CHR_MI_ENC_10)

        if u == tmf.UUID_CHR_MI_ENC_19:
            if hx == "0000000b0100":
                if not self._negotiated:
                    # First attempt triggers 00000401 MTU negotiation just like real 2.1.1_0159
                    loop.call_soon(cb19, self.c_enc19, bytearray.fromhex("000004011400"))
                else:
                    self._awaiting_host_rand = True
                    loop.call_soon(cb19, self.c_enc19, bytearray.fromhex("00000101"))
            elif hx == "000005011400":
                self._negotiated = True
            elif self._awaiting_host_rand and hx.startswith("0100") and len(hx) == 36:
                self._awaiting_host_rand = False
                host_rand_hex = hx[4:]
                dev_info, host_info = tmf.derive_mi_login_keys(
                    self.mi_token_hex, host_rand_hex, self.dev_random.hex()
                )
                self._dev_hmac = dev_info
                self._expected_host_hmac = host_info
                loop.call_soon(
                    cb19, self.c_enc19, bytearray.fromhex("0000020d" + self.dev_random.hex())
                )
            elif hx == "00000300" and self._dev_hmac:
                digest = self._dev_hmac
                self._dev_hmac = ""
                loop.call_soon(cb19, self.c_enc19, bytearray.fromhex("0000020c" + digest))
            elif hx == "0000000a0100":
                self._awaiting_host_hmac = True
                loop.call_soon(cb19, self.c_enc19, bytearray.fromhex("00000101"))
            elif self._awaiting_host_hmac and hx.startswith("0100"):
                self._awaiting_host_hmac = False
                recv_hmac = hx[4:]
                loop.call_soon(cb19, self.c_enc19, bytearray.fromhex("00000100"))
                if recv_hmac == self._expected_host_hmac:
                    loop.call_soon(cb10, self.c_enc10, bytearray.fromhex("21000000"))
                else:
                    loop.call_soon(cb10, self.c_enc10, bytearray.fromhex("23000000"))

    async def disconnect(self) -> None:
        self.is_connected = False


class TestCryptoAndChecksums(unittest.TestCase):
    def test_normalize_mac_and_credential_validation(self) -> None:
        self.assertEqual(tmf.normalize_mac("a4:c1:38:11:22:33"), "A4:C1:38:11:22:33")
        self.assertEqual(tmf.normalize_mac("a4c138-112233"), "A4:C1:38:11:22:33")
        with self.assertRaises(ValueError):
            tmf.normalize_mac("A4:C1:38")

        tk, bk = tmf.validate_mi_credentials("00112233445566778899AABB", "00112233445566778899aabbccddeeff")
        self.assertEqual(tk, "00112233445566778899aabb")
        self.assertEqual(bk, "00112233445566778899aabbccddeeff")

        # Swapped 32-hex bind key passed as --token must produce clear error
        with self.assertRaisesRegex(ValueError, "Bind Key"):
            tmf.validate_mi_credentials("00112233445566778899aabbccddeeff")

    def test_hkdf_sha256_rfc5869_case1(self) -> None:
        ikm = bytes([0x0B] * 22)
        salt = bytes.fromhex("000102030405060708090a0b0c")
        info = bytes.fromhex("f0f1f2f3f4f5f6f7f8f9")
        expected = bytes.fromhex(
            "3cb25f25faacd57a90434f64d0362f2a"
            "2d2d0a90cf1a5a4c5db02d56ecc4c5bf"
            "34007208d5b887185865"
        )
        self.assertEqual(tmf.hkdf_sha256(ikm, 42, salt, info), expected)

    def test_telink_crc32_is_uninverted_and_ota_packets_match_js(self) -> None:
        sample = b"123456789"
        # Standard CRC32("123456789") == 0xCBF43926 -> un-inverted == 0x340BC6D9
        self.assertEqual(tmf.telink_crc32(sample), 0x340BC6D9)

        # Modbus CRC16("123456789") == 0x4B37
        self.assertEqual(tmf.crc16_modbus(sample), 0x4B37)

        pkt = tmf.build_ota_block_packet(0x0102, bytes(range(16)))
        self.assertEqual(len(pkt), 20)
        self.assertEqual(pkt[:2], b"\x02\x01")  # little-endian block count
        expected_crc = tmf.crc16_modbus(pkt[:18])
        self.assertEqual(struct.unpack_from("<H", pkt, 18)[0], expected_crc)

        end_pkt = tmf.build_ota_end_packet(0x0500)
        # last_idx = 0x04FF, ~last_idx = 0xFB00 -> 02 ff ff 04 00 fb
        self.assertEqual(end_pkt.hex(), "02ffff0400fb")


class TestFirmwareAndSignature2110159(unittest.TestCase):
    def test_inspect_and_sign_211_0159_image(self) -> None:
        raw_fw = _make_valid_telink_bin(body_blocks=80)  # hsize = 1284 (0x504)
        patches = ((0x0010, 0x00012313), (0x0024, 0x000124A7))
        sign_bin = _make_synthetic_sign_file(cl=80, fill_len=64, patches=patches)

        ota_image, info, sig_applied = tmf.prepare_ota_image(raw_fw, sign_data=sign_bin)
        self.assertTrue(sig_applied)
        self.assertFalse(info.already_signed)
        self.assertEqual(len(ota_image) % 16, 0)

        # Verify layout produced by append_signature_211_0159:
        # [raw_fw] + 00000000 + cdata[4:cl] (with fwsize patched at cdata[0x14..0x18], i.e. raw_fw_len + 0x14) + xdata
        fw_len = len(raw_fw)
        self.assertEqual(ota_image[:fw_len], raw_fw)
        self.assertEqual(ota_image[fw_len : fw_len + 12], tmf.MITFOTA_HEADER)
        patched_fwsize = struct.unpack_from("<I", ota_image, fw_len + 0x14)[0]
        self.assertEqual(patched_fwsize, fw_len)

        xdata_offset = fw_len + 4 + (80 - 4)
        xdata = ota_image[xdata_offset : xdata_offset + 64]
        self.assertEqual(struct.unpack_from("<I", xdata, 0x10)[0], 0x00012313)
        self.assertEqual(struct.unpack_from("<I", xdata, 0x24)[0], 0x000124A7)
        self.assertEqual(xdata[:0x10], b"\xff" * 0x10)

    def test_already_signed_mitfota_skips_double_signing(self) -> None:
        raw_fw = _make_valid_telink_bin(body_blocks=80)
        already_signed = raw_fw + tmf.MITFOTA_HEADER + (b"\x77" * 1344)
        sign_bin = _make_synthetic_sign_file()

        ota_image, info, sig_applied = tmf.prepare_ota_image(already_signed, sign_data=sign_bin)
        self.assertTrue(info.already_signed)
        self.assertFalse(sig_applied)
        self.assertEqual(ota_image[: len(already_signed)], already_signed)

    def test_corrupt_crc_rejected(self) -> None:
        bad = bytearray(_make_valid_telink_bin(body_blocks=80))
        bad[0x40] ^= 0xFF
        with self.assertRaisesRegex(tmf.FlasherError, "Incorrect CRC"):
            tmf.inspect_ota_firmware(bytes(bad))


class TestMiLoginStateMachine(unittest.TestCase):
    def test_mible_v1_chunked_login_path(self) -> None:
        token = "112233445566778899aabbcc"
        host_rand = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
        dev_rand = bytes.fromhex("f0e0d0c0b0a090807060504030201000")
        expected_dev_info, expected_host_send = tmf.derive_mi_login_keys(
            token, host_rand.hex(), dev_rand.hex()
        )

        sess = tmf.MiLoginSession(token, host_random_bytes=host_rand)
        init_writes = sess.start_login()
        self.assertEqual([(w.target, w.data.hex()) for w in init_writes], [
            ("enc_10", "24000000"),
            ("enc_19", "0000000b0100"),
        ])

        w1 = sess.on_enc_19(bytes.fromhex("00000101"))
        self.assertEqual(w1[0].data.hex(), "0100" + host_rand.hex())

        w2 = sess.on_enc_19(bytes.fromhex("0000000d0100"))
        self.assertEqual(w2[0].data.hex(), "00000101")

        w3 = sess.on_enc_19(bytes.fromhex("0100" + dev_rand.hex()))
        self.assertEqual(w3[0].data.hex(), "00000100")

        w4 = sess.on_enc_19(bytes.fromhex("0000000c0200"))
        self.assertEqual(w4[0].data.hex(), "00000101")

        self.assertEqual(sess.on_enc_19(bytes.fromhex("0100" + expected_dev_info[:36])), [])
        w5 = sess.on_enc_19(bytes.fromhex("0200" + expected_dev_info[36:]))
        self.assertEqual([w.data.hex() for w in w5], ["00000100", "0000000a0200"])

        w6 = sess.on_enc_19(bytes.fromhex("00000101"))
        self.assertEqual(
            [w.data.hex() for w in w6],
            ["0100" + expected_host_send[:36], "0200" + expected_host_send[36:72]],
        )

        sess.on_enc_10(bytes.fromhex("21000000"))
        self.assertTrue(sess.is_logged_in)
        self.assertIsNone(sess.error)

    def test_not_registered_error_on_enc_10(self) -> None:
        sess = tmf.MiLoginSession("112233445566778899aabbcc")
        sess.start_login()
        sess.on_enc_10(bytes.fromhex("e0000000"))
        self.assertFalse(sess.is_logged_in)
        self.assertIn("NOT registered", sess.error or "")


class TestTokensFileParser(unittest.TestCase):
    def test_parse_xiaomi_cloud_extractor_json_and_text(self) -> None:
        sample_json = [
            {
                "server": "de",
                "homes": [
                    {
                        "home_id": 123,
                        "devices": [
                            {
                                "name": "Wohnzimmer",
                                "did": "blt.3.1abc",
                                "mac": "a4:c1:38:aa:bb:cc",
                                "token": "0123456789abcdef01234567",
                                "model": "miaomiaoce.sensor_ht.t2",
                                "BLE_DATA": {"beaconkey": "00112233445566778899aabbccddeeff"},
                            }
                        ],
                    }
                ],
            }
        ]
        sample_text = (
            "Devices found for server \"de\":\n"
            "   \x1b[34m---------\x1b[0m\n"
            "   \x1b[33mNAME:     \x1b[0mSchlafzimmer\n"
            "   \x1b[33mID:       \x1b[0mblt.3.2def\n"
            "   \x1b[33mBLE KEY:  \x1b[0mffeeddccbbaa99887766554433221100\n"
            "   \x1b[33mMAC:      \x1b[0mA4:C1:38:DD:EE:FF\n"
            "   \x1b[33mTOKEN:    \x1b[0mfedcba9876543210fedcba98\n"
            "   \x1b[33mMODEL:    \x1b[0mmiaomiaoce.sensor_ht.t2\n"
            "   \x1b[34m---------\x1b[0m\n"
        )

        with tempfile.TemporaryDirectory() as tmp:
            jp = Path(tmp) / "tokens.json"
            tp = Path(tmp) / "tokens.txt"
            jp.write_text(json.dumps(sample_json), encoding="utf-8")
            tp.write_text(sample_text, encoding="utf-8")

            c_json = tmf.parse_tokens_file(jp)
            self.assertEqual(len(c_json), 1)
            self.assertEqual(c_json[0].mac, "A4:C1:38:AA:BB:CC")
            self.assertEqual(c_json[0].token, "0123456789abcdef01234567")
            self.assertEqual(c_json[0].bind_key, "00112233445566778899aabbccddeeff")

            c_txt = tmf.parse_tokens_file(tp)
            self.assertEqual(len(c_txt), 1)
            matched = tmf.lookup_credentials_for_mac(c_txt, "a4c138ddeeff")
            self.assertIsNotNone(matched)
            assert matched is not None
            self.assertEqual(matched.name, "Schlafzimmer")
            self.assertEqual(matched.token, "fedcba9876543210fedcba98")
            self.assertEqual(matched.bind_key, "ffeeddccbbaa99887766554433221100")


class TestEndToEndBle2110159Mock(unittest.TestCase):
    def test_full_211_0159_probe_login_and_ota_stream(self) -> None:
        token = "0123456789abcdef01234567"
        fake_client = _FakeMi211DeviceClient(mi_token_hex=token, hw_rev="B1.6", fw_rev="2.1.1_0159")
        raw_fw = _make_valid_telink_bin(body_blocks=64)  # 1028 bytes
        sign_bin = _make_synthetic_sign_file(cl=80, fill_len=64)
        ota_image, _, sig_applied = tmf.prepare_ota_image(raw_fw, sign_data=sign_bin)
        self.assertTrue(sig_applied)

        async def _scenario() -> tmf.DeviceProfile:
            prof = await tmf.probe_device_profile(fake_client, "A4:C1:38:11:22:33", "LYWSD03MMC")
            await tmf.perform_mi_login(fake_client, mi_token_hex=token, log=lambda _s: None)
            await tmf.stream_telink_ota(
                fake_client,
                ota_image=ota_image,
                mi_enabled=prof.mi_enabled,
                block_delay_s=0.0,
                post_end_wait_s=0.0,
                show_progress=False,
                log=lambda _s: None,
            )
            return prof

        prof = asyncio.run(_scenario())
        self.assertTrue(prof.is_fw_211_0159)
        self.assertTrue(prof.is_2025_hw_variant)
        self.assertEqual(prof.required_sign_file, "sign_2.1.1_0159c.bin")
        self.assertEqual(prof.hwver_id, 3)
        self.assertAlmostEqual(prof.temperature_c or 0.0, 22.45, places=2)

        # Verify fast connection speed request + Telink OTA start/blocks/end framing
        self.assertEqual(fake_client.speed_writes, [bytes.fromhex("1e0000")])
        expected_blocks = len(ota_image) // 16
        # 2 start packets (00ff, 01ff) + N data blocks + 1 end packet (02ff...)
        self.assertEqual(len(fake_client.ota_writes), 2 + expected_blocks + 1)
        self.assertEqual(fake_client.ota_writes[0], (b"\x00\xff", False))
        self.assertEqual(fake_client.ota_writes[1], (b"\x01\xff", False))
        self.assertEqual(fake_client.ota_writes[-1][0], tmf.build_ota_end_packet(expected_blocks))

    def test_bluez_hyphenated_mac_alias_still_detects_lywsd03mmc_211_0159(self) -> None:
        fake_client = _FakeMi211DeviceClient(
            mi_token_hex="0123456789abcdef01234567", hw_rev="B1.5", fw_rev="2.1.1_0159"
        )
        prof = asyncio.run(
            tmf.probe_device_profile(fake_client, "A4:C1:38:11:22:33", "A4-C1-38-11-22-33")
        )
        self.assertEqual(prof.name, "LYWSD03MMC")
        self.assertTrue(prof.is_lywsd03mmc)
        self.assertTrue(prof.is_fw_211_0159)
        self.assertEqual(prof.hwver_id, 10)
        self.assertEqual(prof.required_sign_file, "sign_2.1.1_0159c.bin")

    def test_ota_aborts_on_device_crc_error_status(self) -> None:
        token = "0123456789abcdef01234567"
        fake_client = _FakeMi211DeviceClient(mi_token_hex=token, ota_error_at_read=2)
        raw_fw = _make_valid_telink_bin(body_blocks=64)
        ota_image, _, _ = tmf.prepare_ota_image(raw_fw)

        with self.assertRaisesRegex(tmf.FlasherError, "CRC error in data"):
            asyncio.run(
                tmf.stream_telink_ota(
                    fake_client,
                    ota_image=ota_image,
                    block_delay_s=0.0,
                    post_end_wait_s=0.0,
                    show_progress=False,
                    log=lambda _s: None,
                )
            )


if __name__ == "__main__":
    unittest.main()
