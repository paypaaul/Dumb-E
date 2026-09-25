#include "net.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"

static const char *TAG = "net";

#define NVS_NAMESPACE     "net"
#define BACKOFF_MIN_MS    1000
#define BACKOFF_MAX_MS    30000

static struct {
    bool started;
    bool configured;
    volatile bool connected;
    char ssid[NET_SSID_MAX_LEN + 1];
    char ip[16];
    uint32_t backoff_ms;
    esp_timer_handle_t retry_timer;
    portMUX_TYPE lock;
} s_n = {.lock = portMUX_INITIALIZER_UNLOCKED};

static void retry_cb(void *arg)
{
    (void)arg;
    esp_wifi_connect();
}

static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (s_n.configured) {
            esp_wifi_connect();
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_n.connected = false;
        if (s_n.configured) {
            ESP_LOGW(TAG, "disconnected, retry in %" PRIu32 " ms", s_n.backoff_ms);
            esp_timer_stop(s_n.retry_timer);
            esp_timer_start_once(s_n.retry_timer, (uint64_t)s_n.backoff_ms * 1000u);
            s_n.backoff_ms = s_n.backoff_ms * 2 > BACKOFF_MAX_MS ? BACKOFF_MAX_MS : s_n.backoff_ms * 2;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)data;
        portENTER_CRITICAL(&s_n.lock);
        snprintf(s_n.ip, sizeof(s_n.ip), IPSTR, IP2STR(&event->ip_info.ip));
        portEXIT_CRITICAL(&s_n.lock);
        s_n.connected = true;
        s_n.backoff_ms = BACKOFF_MIN_MS;
        ESP_LOGI(TAG, "connected, IP " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static esp_err_t load_credentials(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READONLY, &h), TAG, "no credentials");
    esp_err_t err = nvs_get_str(h, "ssid", ssid, &ssid_len);
    if (err == ESP_OK) {
        err = nvs_get_str(h, "pass", pass, &pass_len);
    }
    nvs_close(h);
    return err;
}

static esp_err_t apply_credentials(const char *ssid, const char *pass)
{
    wifi_config_t wc = {0};
    /* Fields are fixed-size and not necessarily NUL-terminated: copy with explicit bounds. */
    memcpy(wc.sta.ssid, ssid, strnlen(ssid, sizeof(wc.sta.ssid)));
    memcpy(wc.sta.password, pass, strnlen(pass, sizeof(wc.sta.password)));
    wc.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wc), TAG, "set config");
    portENTER_CRITICAL(&s_n.lock);
    strlcpy(s_n.ssid, ssid, sizeof(s_n.ssid));
    s_n.ip[0] = '\0';
    portEXIT_CRITICAL(&s_n.lock);
    s_n.configured = true;
    s_n.backoff_ms = BACKOFF_MIN_MS;
    return ESP_OK;
}

esp_err_t net_start(void)
{
    ESP_RETURN_ON_FALSE(!s_n.started, ESP_ERR_INVALID_STATE, TAG, "already started");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif");
    esp_netif_create_default_wifi_sta();
    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "storage");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL, NULL),
                        TAG, "handler");
    ESP_RETURN_ON_ERROR(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL, NULL), TAG,
        "handler");
    const esp_timer_create_args_t targs = {.callback = retry_cb, .name = "wifi_retry"};
    ESP_RETURN_ON_ERROR(esp_timer_create(&targs, &s_n.retry_timer), TAG, "timer");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "mode");

    char ssid[NET_SSID_MAX_LEN + 1] = {0};
    char pass[NET_PASS_MAX_LEN + 1] = {0};
    if (load_credentials(ssid, sizeof(ssid), pass, sizeof(pass)) == ESP_OK && ssid[0]) {
        ESP_RETURN_ON_ERROR(apply_credentials(ssid, pass), TAG, "credentials");
    } else {
        ESP_LOGI(TAG, "no WiFi credentials stored (use the `wifi` command)");
    }
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start");
    s_n.started = true;
    return ESP_OK;
}

esp_err_t net_set_credentials(const char *ssid, const char *password)
{
    ESP_RETURN_ON_FALSE(s_n.started, ESP_ERR_INVALID_STATE, TAG, "not started");
    ESP_RETURN_ON_FALSE(ssid != NULL && password != NULL && ssid[0] != '\0' && strlen(ssid) <= NET_SSID_MAX_LEN &&
                            strlen(password) <= NET_PASS_MAX_LEN,
                        ESP_ERR_INVALID_ARG, TAG, "invalid credentials");
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h), TAG, "nvs open");
    esp_err_t err = nvs_set_str(h, "ssid", ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(h, "pass", password);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    ESP_RETURN_ON_ERROR(err, TAG, "nvs write");

    esp_timer_stop(s_n.retry_timer);
    esp_wifi_disconnect();
    ESP_RETURN_ON_ERROR(apply_credentials(ssid, password), TAG, "credentials");
    return esp_wifi_connect();
}

void net_get_status(net_status_t *status)
{
    memset(status, 0, sizeof(*status));
    status->configured = s_n.configured;
    status->connected = s_n.connected;
    portENTER_CRITICAL(&s_n.lock);
    strlcpy(status->ssid, s_n.ssid, sizeof(status->ssid));
    strlcpy(status->ip, s_n.ip, sizeof(status->ip));
    portEXIT_CRITICAL(&s_n.lock);
}
