#pragma once

#include "esp.h"

#define ESP_RX_BUFFER_SIZE 2048  // 增加到与SDIO一致的大小
#define USB_RX_URB_COUNT 30       // 增加URB数量提高接收性能

struct esp_usb_context
{
    struct esp_adapter *adapter;
    struct usb_device *udev;
    int in_pipe;
    int out_pipe;
    struct sk_buff_head tx_q[MAX_PRIORITY_QUEUES];
    struct sk_buff_head rx_q[MAX_PRIORITY_QUEUES];
    
    // 异步接收相关
    struct urb *rx_urb[USB_RX_URB_COUNT];
    u8 *rx_buffer[USB_RX_URB_COUNT];
    spinlock_t rx_lock;
    atomic_t rx_active;
};

enum
{
    CLOSE_DATAPATH,
    OPEN_DATAPATH,
};
