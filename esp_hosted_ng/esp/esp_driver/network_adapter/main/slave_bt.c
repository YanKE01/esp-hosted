// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#ifdef CONFIG_BT_ENABLED
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/lldesc.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "slave_bt.h"

#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32S3)
#include "esp_private/gdma.h"
#endif

static const char BT_TAG[] = "FW_BT";
extern QueueHandle_t to_host_queue[MAX_PRIORITY_QUEUES];

/* ***** HCI specific part ***** */

#define VHCI_MAX_TIMEOUT_MS     2000
static SemaphoreHandle_t vhci_send_sem;

static void controller_rcv_pkt_ready(void)
{
    if (vhci_send_sem) {
        xSemaphoreGive(vhci_send_sem);
    }
}

static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
    esp_err_t ret = ESP_OK;
    interface_buffer_handle_t buf_handle;
    uint8_t *buf = NULL;

    buf = (uint8_t *) malloc(len);

    if (!buf) {
        ESP_LOGE(BT_TAG, "HCI Send packet: memory allocation failed");
        return ESP_FAIL;
    }

    memcpy(buf, data, len);

    memset(&buf_handle, 0, sizeof(buf_handle));

    buf_handle.if_type = ESP_HCI_IF;
    buf_handle.if_num = 0;
    buf_handle.payload_len = len;
    buf_handle.payload = buf;
    buf_handle.wlan_buf_handle = buf;
    buf_handle.free_buf_handle = free;

#if CONFIG_ESP_BT_DEBUG
    ESP_LOG_BUFFER_HEXDUMP("bt_tx", data, len, ESP_LOG_INFO);
    ESP_LOGI("bt_tx","payload_len: %d", len);
#endif
    ret = xQueueSend(to_host_queue[PRIO_Q_MID], &buf_handle, portMAX_DELAY);

    if (ret != pdTRUE) {
        ESP_LOGE(BT_TAG, "HCI send packet: Failed to send buffer\n");
        free(buf);
        return ESP_FAIL;
    }

    return 0;
}

static esp_vhci_host_callback_t vhci_host_cb = {
    controller_rcv_pkt_ready,
    host_rcv_pkt
};

void process_hci_rx_pkt(uint8_t *payload, uint16_t payload_len)
{
    /* VHCI needs one extra byte at the start of payload */
    /* that is accomodated in esp_payload_header */
#if CONFIG_ESP_BT_DEBUG
    ESP_LOG_BUFFER_HEXDUMP("bt_rx", payload, payload_len, ESP_LOG_INFO);
#endif
    payload--;
    payload_len++;

    if (!esp_vhci_host_check_send_available()) {
        ESP_LOGI(BT_TAG, "VHCI not available");
    }

#if SOC_ESP_NIMBLE_CONTROLLER
    esp_vhci_host_send_packet(payload, payload_len);
#else
    if (vhci_send_sem) {
        if (xSemaphoreTake(vhci_send_sem, VHCI_MAX_TIMEOUT_MS) == pdTRUE) {
            esp_vhci_host_send_packet(payload, payload_len);
            ESP_LOGI(BT_TAG, "esp_vhci_host_send_packet: %d", payload_len);
        } else {
            ESP_LOGI(BT_TAG, "VHCI sem timeout");
        }
    }
#endif
}


esp_err_t initialise_bluetooth(void)
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    esp_err_t ret = ESP_OK;

    ret = esp_vhci_host_register_callback(&vhci_host_cb);

    if (ret != ESP_OK) {
        ESP_LOGE(BT_TAG, "Failed to register VHCI callback");
        return ret;
    }

    vhci_send_sem = xSemaphoreCreateBinary();
    if (vhci_send_sem == NULL) {
        ESP_LOGE(BT_TAG, "Failed to create VHCI send sem");
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreGive(vhci_send_sem);

    return ESP_OK;
}

void deinitialize_bluetooth(void)
{
    if (vhci_send_sem) {
        /* Dummy take and give sema before deleting it */
        xSemaphoreTake(vhci_send_sem, portMAX_DELAY);
        xSemaphoreGive(vhci_send_sem);
        vSemaphoreDelete(vhci_send_sem);
        vhci_send_sem = NULL;
    }
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
}

uint8_t get_bluetooth_capabilities(void)
{
    uint8_t cap = 0;
    ESP_LOGI(BT_TAG, "- BT/BLE");
// #if 1
// #if CONFIG_ESP_SPI_HOST_INTERFACE
    // ESP_LOGI(BT_TAG, "   - HCI Over SPI");
    // cap |= ESP_BT_SPI_SUPPORT;
// #else
    ESP_LOGI(BT_TAG, "   - HCI Over SDIO");
    cap |= ESP_BT_SDIO_SUPPORT;
// #endif
// #elif BLUETOOTH_UART
//     ESP_LOGI(BT_TAG, "   - HCI Over UART");
//     cap |= ESP_BT_UART_SUPPORT;
// #endif

    ESP_LOGI(BT_TAG, "   - BLE only");
    cap |= ESP_BLE_ONLY_SUPPORT;

    return cap;
}

#endif
