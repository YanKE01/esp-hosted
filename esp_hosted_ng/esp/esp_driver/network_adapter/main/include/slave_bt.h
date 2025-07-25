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
//
#ifndef __SLAVE_BT_H__
#define __SLAVE_BT_H__

#ifdef CONFIG_BT_ENABLED

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
#include "driver/periph_ctrl.h"
#define DISABLE_INTR_ON_GPIO GPIO_PIN_INTR_DISABLE
#else
#include "esp_private/periph_ctrl.h"
#define DISABLE_INTR_ON_GPIO GPIO_INTR_DISABLE
#endif

/* only BLE for chipsets other than ESP32 */
#define BLUETOOTH_BLE      1
#define BLUETOOTH_HCI    4

void process_hci_rx_pkt(uint8_t *payload, uint16_t payload_len);


void deinitialize_bluetooth(void);
esp_err_t initialise_bluetooth(void);
uint8_t get_bluetooth_capabilities(void);

#endif /* CONFIG_BT_ENABLED */

#endif /* __SLAVE_BT_H__ */
