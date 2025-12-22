// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 */

#pragma once

#include "esp.h"

#define ESP_RX_BUFFER_SIZE 1920

struct esp_usb_context {
    struct esp_adapter *adapter;
    struct usb_device *udev;
    int in_pipe;
    int out_pipe;
    struct sk_buff_head tx_q[MAX_PRIORITY_QUEUES];
    struct sk_buff_head rx_q[MAX_PRIORITY_QUEUES];

    // Asynchronous RX related
    struct urb *rx_urb;
    uint8_t *rx_buffer;
    int usb_rx_pipe_status;
    struct work_struct rx_work;
    int rx_urb_failed_count;
};

enum {
    CLOSE_DATAPATH,
    OPEN_DATAPATH,
};
