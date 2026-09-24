/*
 * Serial command protocol v0 (see docs/protocol.md), implemented with esp_console on UART0.
 *
 * Every command ends with exactly one result line: `ok [key=value ...]` or `err <code> <message>`.
 * Other lines (logs, `#` comments) must be ignored by clients.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Registers the commands and starts the REPL task. */
esp_err_t comms_start(void);

#ifdef __cplusplus
}
#endif
