import binascii
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest


def record(kind, address, width, data=b""):
    body = address.to_bytes(width, "big") + data
    body = bytes([len(body) + 1]) + body
    return "S" + str(kind) + (body + bytes([~sum(body) & 255])).hex().upper()


class TestSRecordPacking(unittest.TestCase):
    def pack(self, directory, records):
        source = Path(directory) / "input.s19"
        source.write_text("\n".join(records))
        output = Path(directory) / "output.dfu"
        return subprocess.run(
            [sys.executable, str(Path(__file__).with_name("dfuse-pack.py")), "-s", str(source), str(output)],
            capture_output=True,
        ), output

    def test_preserves_addresses_bytes_name_and_checksum(self):
        records = [record(0, 0, 2, b"sample.s19"), record(1, 0, 2, b"ab"), record(1, 2, 2, b"c"),
                   record(2, 0x120000, 3, b"d"), record(3, 0x08000000, 4, b"e"), record(5, 4, 2), record(7, 0x08000000, 4)]
        with tempfile.TemporaryDirectory() as directory:
            result, output = self.pack(directory, records)
            self.assertEqual(result.returncode, 0, result.stderr)
            data = output.read_bytes()
            self.assertEqual(data[22:29], b"sample\0")
            self.assertEqual(struct.unpack_from("<I", data, 281)[0], 3)
            offset = 285
            for address, expected in [(0, b"abc"), (0x120000, b"d"), (0x08000000, b"e")]:
                self.assertEqual(struct.unpack_from("<II", data, offset), (address, len(expected)))
                offset += 8
                self.assertEqual(data[offset:offset + len(expected)], expected)
                offset += len(expected)
            self.assertEqual(offset + 16, len(data))
            self.assertEqual(struct.unpack_from("<I", data, len(data) - 4)[0], ~binascii.crc32(data[:-4]) & 0xffffffff)

    def test_rejects_bad_and_empty_inputs_before_writing(self):
        valid = record(1, 0, 2, b"ab")
        with tempfile.TemporaryDirectory() as directory:
            for bad in [valid[:-2] + "FF", valid[:2] + "FE" + valid[4:], record(0, 0, 2, b"name"), valid[:-1] + "Z"]:
                result, output = self.pack(directory, [bad])
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
