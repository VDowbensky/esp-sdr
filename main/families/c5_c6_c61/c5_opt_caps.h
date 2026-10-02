/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
 *
 * ESPsoup ESP32-C5 extensions: version, CAPS tokens and LIMITS? keys.
 * Included by receiver.c before burst_limits.h.  Every token names commands
 * that stock clients never send; existing commands and replies are unchanged.
 * See docs/espsoup-c5.md. */
#pragma once
#define FWOPT_VERSION "0.7.0"
#define CAP_MAX_WORDS 32704u   /* BIGCAP: two-half dump, see c5_engine.h */
#define C5_OPT_CAPS " BIGCAP CAPT TEMP FREQK FWOPT DDC SPECDB SWEEP"
#define LIMITS_EXTRA ",\"maxSamples\":32704" \
    ",\"ddc\":{\"src\":[80000000,40000000,20000000],\"bits\":[8,16]},\"spec\":{\"nfft\":[64,1024]},\"freqStepHz\":1000"
