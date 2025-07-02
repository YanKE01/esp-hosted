#pragma once

#include "esp.h"

struct esp_usb_context
{
    struct esp_adapter *adapter;
    struct usb_device *udev;
    int in_pipe;
    int out_pipe;
    struct sk_buff_head tx_q[MAX_PRIORITY_QUEUES];
    struct sk_buff_head rx_q[MAX_PRIORITY_QUEUES];
    struct workqueue_struct *usb_workqueue;
    struct work_struct usb_tx_work;
    struct work_struct usb_rx_work;
    
    /* Async USB receive support */
    struct urb *rx_urb;
    u8 *rx_buffer;
    int rx_buffer_size;
    int is_alive;
    struct mutex rx_lock;
};

enum
{
    CLOSE_DATAPATH,
    OPEN_DATAPATH,
};
