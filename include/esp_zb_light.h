#pragma once
#include "esp_zigbee_core.h"

#define INSTALLCODE_POLICY_ENABLE       false
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000
#define HA_ESP_LIGHT_ENDPOINT           10
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

#define ESP_MANUFACTURER_NAME "\x09""ESPRESSIF"
#define ESP_MODEL_IDENTIFIER "\x07"CONFIG_IDF_TARGET

#define ESP_ZB_ZED_CONFIG() { \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED, \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
        .nwk_cfg.zed_cfg = { \
            .ed_timeout = ED_AGING_TIMEOUT, \
            .keep_alive = ED_KEEP_ALIVE, \
        }, \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG() { \
        .radio_mode = ZB_RADIO_MODE_NATIVE, \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG() { \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, \
    }
