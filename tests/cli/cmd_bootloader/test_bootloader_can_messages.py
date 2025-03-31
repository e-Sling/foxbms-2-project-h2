#!/usr/bin/env python3
#
# Copyright (c) 2010 - 2025, Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# We kindly request you to use one or more of the following phrases to refer to
# foxBMS in your hardware, software, documentation or advertising materials:
#
# - "This product uses parts of foxBMS®"
# - "This product includes parts of foxBMS®"
# - "This product is derived from foxBMS®"

"""Testing file 'cli/cmd_bootloader/bootloader_can_messages.py'."""

import logging
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

try:
    from cli.cmd_bootloader.bootloader_can_messages import (
        Messages,
        RequestCode8Bits,
    )
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).parents[3]))
    from cli.cmd_bootloader.bootloader_can_messages import Messages, RequestCode8Bits


@patch.object(logging, "info", return_value=None)
@patch.object(logging, "warning", return_value=None)
@patch.object(logging, "error", return_value=None)
class TestBootloaderCanMessages(unittest.TestCase):
    """Class to test the class BootloaderCanMessages."""

    def setUp(self):
        self.messages = Messages()

    def test_init(self, *_):
        """Function to test function __init__()."""
        # Case 1: the input dbc file is not valid
        with self.assertRaises(SystemExit) as cm:
            Messages(dbc_file=Path("fake"))
        self.assertEqual(
            cm.exception.code, "The provided dbc file 'fake' does not exist."
        )

        with self.assertRaises(SystemExit) as cm:
            Messages(dbc_file=Path(Path(__file__).parent / "invalid.dbc"))
        self.assertEqual(
            cm.exception.code, "There are no messages in the database file."
        )

    def test_get_message(self, *_):
        """Function to test function _get_message()."""
        # Case 2: cannot find the message in the dbc file
        with self.assertRaises(SystemExit) as cm:
            self.messages._get_message(name="fake")  # pylint: disable=protected-access
        self.assertEqual(
            cm.exception.code,
            "The name of message 'fake' cannot be found in the CAN database.",
        )

        # Case 1: cannot find the signal in the given message
        with self.assertRaises(SystemExit) as cm:
            self.messages._get_message(  # pylint: disable=protected-access
                name="f_BootloaderActionRequest", fake="fake_value"
            )
        self.assertEqual(
            cm.exception.code,
            "Cannot find the signal 'fake' in CAN message 'f_BootloaderActionRequest'.",
        )

        # Case 2: the range has surpassed the range limit
        with self.assertRaises(SystemExit) as cm:
            # pylint: disable-next=protected-access
            self.messages._get_message(name="f_Crc8Bytes", Crc=0xFFFFFFFFFFFFFFFFF)
        self.assertEqual(
            cm.exception.code,
            "The value of the signal 'Crc' is out of range.",
        )

        # Case 3: the value of one signal is not in the corresponding enum
        with self.assertRaises(SystemExit) as cm:
            self.messages._get_message(  # pylint: disable=protected-access
                name="f_BootloaderActionRequest", RequestCode8Bits=100
            )
        self.assertEqual(
            cm.exception.code,
            "The value of the signal 'RequestCode8Bits' is not in the corresponding enum.",
        )

    def test_check_range(self, *_):
        """Function to test function _check_range()."""
        msg = self.messages.db.get_message_by_name("f_Data8Bytes")
        signal = msg.get_signal_by_name("Data")
        # pylint: disable=protected-access
        # Input signal_value is in range
        self.assertTrue(
            self.messages._check_range(signal=signal, signal_value=0xFFFFFFFFFFFFFFFF),
        )
        # Input signal_value is larger than the maximum
        self.assertFalse(
            self.messages._check_range(signal=signal, signal_value=-1),
        )
        # Input signal_value is smaller than the minimum
        self.assertFalse(
            self.messages._check_range(signal=signal, signal_value=0x5FFFFFFFFFFFFFFFF),
        )
        # pylint: enable=protected-access

    def test_check_enum(self, *_):
        """Function to test function _check_enum()."""
        msg = self.messages.db.get_message_by_name("f_BootloaderActionRequest")
        signal = msg.get_signal_by_name("RequestCode8Bits")
        # pylint: disable=protected-access
        # Input signal_value is one of the enum value
        self.assertTrue(
            self.messages._check_enum(signal=signal, signal_value=1),
        )
        # Input signal_value is not one of the enum value
        self.assertFalse(
            self.messages._check_enum(signal=signal, signal_value=100),
        )
        # pylint: enable=protected-access

    def test_get_message_request_msg(self, *_):
        """Function to test function get_message_request_msg()."""
        request_code = RequestCode8Bits.CmdToTransferProgram
        msg = self.messages.get_message_request_msg(request_code)
        self.assertTrue(
            msg == {"Name": "f_BootloaderActionRequest", "RequestCode8Bits": 1}
        )

    def test_get_message_transfer_program_info(self, *_):
        """Function to test function get_message_transfer_program_info()."""
        len_of_program_in_bytes = 100
        num_of_transfer_loops = 200
        msg = self.messages.get_message_transfer_program_info(
            len_of_program_in_bytes, num_of_transfer_loops
        )
        self.assertEqual(
            msg,
            {
                "Name": "f_TransferProcessInfo",
                "ProgramLength": 100,
                "RequiredTransferLoops": 200,
            },
        )

    def test_get_message_data_8_bytes(self, *_):
        """Function to test function test_get_message_data_8_bytes()."""
        data_8_bytes = 0xFFFFFFFFFFFFFFFF
        msg = self.messages.get_message_data_8_bytes(data_8_bytes)
        self.assertEqual(msg, {"Name": "f_Data8Bytes", "Data": 0xFFFFFFFFFFFFFFFF})

    def test_get_message_crc_8_bytes(self, *_):
        """Function to test function test_get_message_crc_8_bytes()."""
        crc_8_bytes = 0xFFFFFFFFFFFFFFFF
        msg = self.messages.get_message_crc_8_bytes(crc_8_bytes)
        self.assertEqual(msg, {"Name": "f_Crc8Bytes", "Crc": 0xFFFFFFFFFFFFFFFF})

    def test_get_message_loop_info(self, *_):
        """Function to test function test_get_message_loop_info()."""
        num_of_loop = 100
        msg = self.messages.get_message_loop_info(num_of_loop)
        self.assertEqual(msg, {"Name": "f_LoopInfo", "LoopNumber": 100})


if __name__ == "__main__":
    unittest.main()
