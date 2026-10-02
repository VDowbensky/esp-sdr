/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
 *
 * ESPsoup receive-only extensions for the ESP32-C5 burst receiver.
 *
 * Everything here is additive: stock commands and replies are unchanged, new
 * behaviour is only reachable through new commands advertised as new CAPS
 * tokens (c5_opt_caps.h).  Receive only: nothing here enables the transmitter.
 *
 *   c5_engine.h  register-level capture engine, BIGCAP, CAPT, FREQK, TEMP
 *   c5_dsp.h     DDC snapshots, SPEC (3 arguments) / SWEEP power spectra
 *   c5_cont.h    continuous engine: STREAM, CSPEC, CPKT, CGRID, CSTAT
 *   c5_demod.h   on-device VFM video and GFSK packet demodulators
 *
 * See docs/espsoup-c5.md for the protocol, measurements and limitations.
 */
#pragma once
#include "c5_engine.h"
#include "c5_dsp.h"
#include "c5_cont.h"
#include "c5_demod.h"

static bool opt_command(char *line) {
    dm_release_unless_vector(line);
    if (!strcmp(line, "FWOPT?")) { reply("FWOPT " FWOPT_VERSION " espsoup rx-only C5\n"); return true; }
    return engine_command(line) || dsp_command(line) || cont_command(line) || demod_command(line);
}
