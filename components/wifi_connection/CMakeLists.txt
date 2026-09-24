idf_component_register(SRCS "wifi_connection.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver esp_wifi esp_event nvs_flash lwip wifi_connection)