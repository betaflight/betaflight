#
# This file is part of Betaflight.
#
# Betaflight is free software. You can redistribute this software
# and/or modify this software under the terms of the GNU General
# Public License as published by the Free Software Foundation,
# either version 3 of the License, or (at your option) any later
# version.
#
# Betaflight is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public
# License along with this software.
#
# If not, see <http://www.gnu.org/licenses/>.
#

import binascii
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest


def record(kind, address, width, data=b""):
    """Encode a test S-record with its byte count and one's-complement checksum."""
    body = address.to_bytes(width, "big") + data
    body = bytes([len(body) + 1]) + body
    return "S" + str(kind) + (body + bytes([~sum(body) & 255])).hex().upper()


class TestSRecordPacking(unittest.TestCase):
    def pack(self, directory, records):
        """Run the S-record packer and return its process result and output path."""
        source = Path(directory) / "input.s19"
        source.write_text("\n".join(records))
        output = Path(directory) / "output.dfu"
        return subprocess.run(
            [sys.executable, str(Path(__file__).with_name("dfuse-pack.py")), "-s", str(source), str(output)],
            capture_output=True,
        ), output

    def test_preserves_addresses_bytes_name_and_checksum(self):
        """Check the target header, coalesced elements, payloads, and DfuSe suffix."""
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
        """Invalid input must fail before the output file is created."""
        valid = record(1, 0, 2, b"ab")
        # Incorrect byte count, but the one's-complement checksum is valid.
        bad_count = "S1FE000061623E"
        with tempfile.TemporaryDirectory() as directory:
            for bad in [valid[:-2] + "FF", bad_count, record(0, 0, 2, b"name"), valid[:-1] + "Z"]:
                result, output = self.pack(directory, [bad])
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(output.exists())

    def test_keeps_header_name_without_suffix(self):
        """An S0 name ending in bare '19' must retain both digits."""
        with tempfile.TemporaryDirectory() as directory:
            result, output = self.pack(directory, [record(0, 0, 2, b"board19"), record(1, 0, 2, b"x")])
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(output.read_bytes()[22:30], b"board19\0")

    def test_data_at_last_address(self):
        """The final 32-bit address can hold one byte, but not two."""
        with tempfile.TemporaryDirectory() as directory:
            result, output = self.pack(directory, [record(3, 0xffffffff, 4, b"x")])
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(struct.unpack_from("<IIB", output.read_bytes(), 285),
                             (0xffffffff, 1, ord("x")))

        with tempfile.TemporaryDirectory() as directory:
            result, output = self.pack(directory, [record(3, 0xffffffff, 4, b"xy")])
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(b"line 1", result.stderr)
            self.assertFalse(output.exists())

    def test_accepts_matching_data_record_counts(self):
        """S5 and S6 count data records, including empty and adjacent records."""
        records = [record(1, 0, 2, b"a"), record(1, 1, 2, b"b"),
                   record(2, 0x120000, 3), record(3, 0x08000000, 4, b"c")]
        for kind, width in [(5, 2), (6, 3)]:
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as directory:
                result, output = self.pack(directory, records + [record(kind, 4, width)])
                self.assertEqual(result.returncode, 0, result.stderr)
                data = output.read_bytes()
                self.assertEqual(struct.unpack_from("<I", data, 281)[0], 2)
                self.assertEqual(struct.unpack_from("<II2s", data, 285), (0, 2, b"ab"))
                self.assertEqual(struct.unpack_from("<II1s", data, 295), (0x08000000, 1, b"c"))

    def test_rejects_incorrect_data_record_counts_before_writing(self):
        """A count mismatch must not produce a partial image."""
        for kind, width, count in [(5, 2, 0), (5, 2, 2), (6, 3, 0), (6, 3, 2), (6, 3, 0x10001)]:
            with self.subTest(kind=kind, count=count), tempfile.TemporaryDirectory() as directory:
                result, output = self.pack(directory, [record(1, 0, 2, b"a"), record(kind, count, width)])
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(b"line 2", result.stderr)
                self.assertFalse(output.exists())

    def test_count_records_only_count_previous_data_records(self):
        """Data after S5 or S6 must not change the count it validates."""
        for kind, width in [(5, 2), (6, 3)]:
            for count in [1, 2]:
                with self.subTest(kind=kind, count=count), tempfile.TemporaryDirectory() as directory:
                    records = [record(1, 0, 2, b"a"), record(kind, count, width), record(1, 1, 2, b"b")]
                    result, output = self.pack(directory, records)
                    if count == 1:
                        self.assertEqual(result.returncode, 0, result.stderr)
                        self.assertEqual(struct.unpack_from("<II2s", output.read_bytes(), 285), (0, 2, b"ab"))
                    else:
                        self.assertNotEqual(result.returncode, 0)
                        self.assertIn(b"line 2", result.stderr)
                        self.assertFalse(output.exists())

    def test_rejects_control_record_data_before_writing(self):
        """A changed record type can hide data without changing its checksum."""
        for kind, data_kind, width in [(5, 1, 2), (6, 2, 3), (7, 3, 4), (8, 2, 3), (9, 1, 2)]:
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as directory:
                data_record = record(data_kind, 1, width, b"b")
                control_record = "S" + str(kind) + data_record[2:]
                result, output = self.pack(directory, [record(1, 0, 2, b"a"), control_record])
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(b"line 2", result.stderr)
                self.assertFalse(output.exists())

    def test_accepts_control_records_without_data(self):
        """S7, S8, and S9 records can set the entry address without payload."""
        for kind, width in [(7, 4), (8, 3), (9, 2)]:
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as directory:
                result, output = self.pack(directory, [record(1, 0, 2, b"a"), record(kind, 0, width)])
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(struct.unpack_from("<II1s", output.read_bytes(), 285), (0, 1, b"a"))


if __name__ == "__main__":
    unittest.main()
