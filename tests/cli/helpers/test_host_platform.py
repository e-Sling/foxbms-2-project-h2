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

"""Testing file 'cli/helpers/host_platform.py'."""

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

try:
    from cli.helpers import host_platform
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).parents[3]))
    from cli.helpers import host_platform


class TestHostPlatform(unittest.TestCase):
    """Class to test the host platform script."""

    @patch("sys.platform", new="linux")
    def test_platform_linux(self):
        """test for Linux."""
        self.assertEqual(host_platform.get_platform(), "linux")

    @patch("sys.platform", new="win32")
    def test_platform_win32(self):
        """test for Windows."""
        self.assertEqual(host_platform.get_platform(), "win32")

    @patch("sys.platform", new="foo")
    def test_platform_unsupported(self):
        """test for unsupported platform."""
        with self.assertRaises(SystemExit) as cm:
            host_platform.get_platform()
        self.assertEqual(cm.exception.code, "Running on an unsupported platform.")


if __name__ == "__main__":
    unittest.main()
