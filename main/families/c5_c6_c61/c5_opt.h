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
 *
 * See docs/espsoup-c5.md for the protocol, measurements and limitations.
 */
#pragma once
#include "c5_engine.h"

static bool opt_command(char *line) {
    if (!strcmp(line, "FWOPT?")) { reply("FWOPT " FWOPT_VERSION " espsoup rx-only C5\n"); return true; }
    return engine_command(line);
}
