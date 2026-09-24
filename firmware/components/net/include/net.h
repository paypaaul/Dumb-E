/*
 * WiFi station. Credentials live in NVS (set with net_set_credentials(), e.g. from the `wifi` command),
 * never in the source. Reconnects with exponential backoff. The WiFi driver keeps its own config in RAM only,
 * so connecting never writes flash while the arm moves.
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NET_SSID_MAX_LEN 32
#define NET_PASS_MAX_LEN 64

typedef struct {
    bool configured;
    bool connected;
    char ssid[NET_SSID_MAX_LEN + 1];
    char ip[16];
} net_status_t;

/* Initializes netif and WiFi (requires NVS and the default event loop) and connects if configured. */
esp_err_t net_start(void);

/* Stores new credentials in NVS and (re)connects. Must not be called while the arm moves (flash write). */
esp_err_t net_set_credentials(const char *ssid, const char *password);

void net_get_status(net_status_t *status);

#ifdef __cplusplus
}
#endif
