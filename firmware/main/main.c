/*
 * Dumb-E firmware entry point: system bring-up only. The arm starts with the drivers disabled and not
 * referenced; everything else is driven through the serial protocol (see docs/protocol.md).
 */
#include "comms.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "net.h"
#include "nvs_flash.h"
#include "robot.h"
#include "sdkconfig.h"

static const char *TAG = "main";

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

void app_main(void)
{
    ESP_ERROR_CHECK(init_nvs());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    const robot_err_t rerr = robot_init();
    if (rerr != ROBOT_OK) {
        ESP_LOGE(TAG, "robot init failed: %s", robot_err_str(rerr));
        abort();
    }

#if CONFIG_DUMBE_NET_ENABLE
    /* The arm is usable without WiFi: a network failure is not fatal. */
    const esp_err_t nerr = net_start();
    if (nerr != ESP_OK) {
        ESP_LOGW(TAG, "WiFi not started: %s", esp_err_to_name(nerr));
    }
#endif

    ESP_ERROR_CHECK(comms_start());
}
