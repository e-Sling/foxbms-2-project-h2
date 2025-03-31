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

"""CAN decode subcommand implementation"""

import sys
from io import TextIOWrapper
from pathlib import Path
from typing import cast

import click
from cantools import database
from cantools.database.can.database import Database

from ..etl.can_decode import CANDecode
from . import read_config


def can_decode_setup(config: Path) -> CANDecode:
    """Reads config file and creates the CANDecode object

    :param config: Path to the configuration file
    :return: CANDecode object
    """
    config_dict = read_config(config)
    validate_decode_config(config_dict)
    config_dict["dbc"] = get_cantools_database(config_dict["dbc"])
    return CANDecode(**config_dict)


def get_cantools_database(dbc_path: Path) -> Database:
    """Loads cantools database and returns it

    :param dbc_path: Path to the dbc file
    :return: Cantools database
    """
    try:
        return cast(
            database.can.database.Database,
            database.load_file(dbc_path, database_format="dbc", encoding="utf-8"),
        )
    except database.UnsupportedDatabaseFormatError:
        click.secho("Invalid DBC file", fg="red", err=True)
        raise SystemExit(1) from None
    except FileNotFoundError:
        click.secho("DBC file not found", fg="red", err=True)
        raise SystemExit(1) from None


def validate_decode_config(config_dict: dict) -> None:
    """Validates the configuration file of the decode subcommand

    :param config_dict: Dictionary with configurations
    """
    if "dbc" not in config_dict:
        click.secho(
            "Configuration file is missing 'dbc' parameter.", fg="red", err=True
        )
        sys.exit(1)
    if not isinstance(config_dict["dbc"], str):
        click.secho(
            "'dbc' in the configuration file is not a string.", fg="red", err=True
        )
        sys.exit(1)
    if "timestamp_pos" not in config_dict:
        click.secho(
            "'Configuration file is missing 'timestamp_pos' parameter'.",
            fg="red",
            err=True,
        )
        sys.exit(1)
    if not isinstance(config_dict["timestamp_pos"], int):
        click.secho("'timestamp_pos' is not an integer.", fg="red", err=True)
        sys.exit(1)
    if "id_pos" not in config_dict:
        click.secho(
            "'Configuration file is missing 'id_pos' parameter'.", fg="red", err=True
        )
        sys.exit(1)
    if not isinstance(config_dict["id_pos"], int):
        click.secho("'id_pos' is not an integer.", fg="red", err=True)
        sys.exit(1)
    if "data_pos" not in config_dict:
        click.secho(
            "'Configuration file is missing 'data_pos' parameter'.", fg="red", err=True
        )
        sys.exit(1)
    if not isinstance(config_dict["data_pos"], int):
        click.secho("'data_pos' is not an integer.", fg="red", err=True)
        sys.exit(1)


def run_decode2stdout(decode_obj: CANDecode) -> None:
    """Executes the can decode step

    :param decode_obj: Object which handles the decoding
    """
    for msg in sys.stdin:
        msg_name, msg_decoded = decode_obj.decode_msg(msg)
        if msg_name and msg_decoded:
            sys.stdout.write(msg_decoded)


def run_decode2file(decode_obj: CANDecode, output_directory: Path) -> None:
    """Executes the can decode step

    :param decode_obj: Object which handles the decoding
    :param output_directory: Directory in which the decoded data should be
        saved
    """
    output_files: dict[str, TextIOWrapper] = {}
    try:
        for msg in sys.stdin:
            msg_name, msg_decoded = decode_obj.decode_msg(msg)
            if msg_name and msg_decoded:
                if msg_name in output_files:
                    output_files[msg_name].write(msg_decoded)
                else:
                    file_name = output_directory / f"{msg_name}.json"
                    f = open(file_name, mode="w", encoding="utf-8")  # pylint: disable=consider-using-with
                    output_files[msg_name] = f
                    f.write(msg_decoded)
    finally:
        for f in output_files.values():
            f.close()
