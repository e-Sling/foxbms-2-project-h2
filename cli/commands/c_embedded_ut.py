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

"""Command line interface definition for Ceedling"""

from typing import get_args

import click

from ..cmd_embedded_ut import embedded_ut_impl
from ..cmd_embedded_ut.embedded_ut_constants import EmbeddedUnitTestVariants
from ..helpers.click_helpers import HELP_NAMES, verbosity_option

CONTEXT_SETTINGS = HELP_NAMES | {"ignore_unknown_options": True}

TESTS = ["app"]


@click.command(context_settings=CONTEXT_SETTINGS)
@click.option(
    "--project", type=click.Choice(get_args(EmbeddedUnitTestVariants)), default="app"
)
@click.argument("ceedling_args", nargs=-1, type=click.UNPROCESSED)
@verbosity_option
@click.pass_context
def ceedling(
    ctx: click.Context,
    project: EmbeddedUnitTestVariants,
    ceedling_args: tuple[str],
    verbose: int = 0,
) -> None:
    """Run the 'ceedling' unit testing tool."""
    if not ceedling_args:
        ceedling_args = ("help",)
    ret = embedded_ut_impl.run_embedded_tests(
        list(ceedling_args), project, stdout=None, stderr=None
    )
    ctx.exit(ret.returncode)
