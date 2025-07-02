#include "utils.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/crc16.h>
#include <linux/kernel.h>
#include "esp_if.h"
#include "esp_api.h"
#include "esp_usb.h"
#include "esp_utils.h"

#define ESP_RX_BUFFER_SIZE 1920
#define TX_MAX_PENDING_COUNT 100
#define TX_RESUME_THRESHOLD (TX_MAX_PENDING_COUNT / 5)

int esp_adjust_spi_clock(struct esp_adapter *adapter, u8 spi_clk_mhz);
int esp_init_interface_layer(struct esp_adapter *adapter, u32 speed);
void esp_deinit_interface_layer(void);
int esp_deinit_module(struct esp_adapter *adapter);
int generate_slave_intr(void *context, u8 data);
int esp_validate_chipset(struct esp_adapter *adapter, u8 chipset);
static int esp_usb_probe(struct usb_interface *interface, const struct usb_device_id *id);
static void esp_usb_disconnect(struct usb_interface *interface);
static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb);
static struct sk_buff *read_packet(struct esp_adapter *adapter);

/* Async USB receive function declarations */
static void esp_usb_rx_complete(struct urb *urb);
static void esp_usb_submit_rx_urb(void);
static void esp_usb_process_rx_data(u8 *data, int len);

volatile u8 host_sleep;
volatile u8 data_path;
struct esp_usb_context usb_context;
static atomic_t tx_pending;
static DEFINE_MUTEX(usb_lock);

static const struct usb_device_id id_table[] = {
    {
        .idVendor = 0xcafe,
        .idProduct = 0x4001,
        .match_flags = USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_PRODUCT,
    },
    {},
};
static struct usb_driver esp_usb_driver = {
    .name = "esp_usb_driver",
    .probe = esp_usb_probe,
    .disconnect = esp_usb_disconnect,
    .id_table = id_table,
};
static struct esp_if_ops if_ops = {
    .read = read_packet,
    .write = write_packet,
};
struct usb_packet_header
{
    uint16_t total_len;
    uint16_t crc16;
} __attribute__((packed));

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len, uint16_t init)
{
    uint16_t crc = init;
    while (len--)
    {
        crc ^= (*data++) << 8;
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* URB completion callback for async USB receive */
static void esp_usb_rx_complete(struct urb *urb)
{
    struct esp_usb_context *context = urb->context;
    
    if (!context || !context->is_alive)
    {
        return;
    }
    
    if (urb->status == 0)
    {
        /* Successfully received data */
        esp_usb_process_rx_data(urb->transfer_buffer, urb->actual_length);
    }
    else if (urb->status == -EPIPE)
    {
        /* Pipe error, clear halt */
        usb_clear_halt(context->udev, context->in_pipe);
        pr_info("esp_usb: Cleared halt on pipe %d\n", context->in_pipe);
    }
    else if (urb->status != -ENOENT && urb->status != -ECONNRESET)
    {
        /* Other errors */
        pr_err("esp_usb: URB error: %d\n", urb->status);
    }
    
    /* Resubmit URB for next receive */
    if (context->is_alive)
    {
        esp_usb_submit_rx_urb();
    }
}

/* Submit URB for async receive */
static void esp_usb_submit_rx_urb(void)
{
    int ret;
    
    if (!usb_context.is_alive || !usb_context.rx_urb || !usb_context.rx_buffer)
    {
        return;
    }
    
    /* Fill URB for bulk receive */
    usb_fill_bulk_urb(usb_context.rx_urb, usb_context.udev, usb_context.in_pipe,
                      usb_context.rx_buffer, usb_context.rx_buffer_size,
                      esp_usb_rx_complete, &usb_context);
    
    /* Submit URB */
    ret = usb_submit_urb(usb_context.rx_urb, GFP_ATOMIC);
    if (ret)
    {
        pr_err("esp_usb: Failed to submit RX URB: %d\n", ret);
        /* Schedule retry after delay */
        if (usb_context.usb_workqueue)
        {
            queue_delayed_work(usb_context.usb_workqueue, 
                              (struct delayed_work *)&usb_context.usb_rx_work, 
                              msecs_to_jiffies(5));
        }
    }
}

/* Process received data */
static void esp_usb_process_rx_data(u8 *data, int len)
{
    struct sk_buff *rx_skb = NULL;
    struct usb_packet_header *header;
    struct esp_payload_header *payload_header;
    u16 expected_len, crc16_received, crc16_calculated;
    u8 priority_queue;
    
    if (!data || len < sizeof(struct usb_packet_header))
    {
        pr_err("esp_usb: Invalid RX data or too small: %d\n", len);
        return;
    }
    
    /* Parse USB packet header */
    header = (struct usb_packet_header *)data;
    expected_len = le16_to_cpu(header->total_len);
    crc16_received = le16_to_cpu(header->crc16);
    
    /* Validate packet length */
    if (expected_len + sizeof(struct usb_packet_header) > len)
    {
        pr_err("esp_usb: Length mismatch: expected %lu, got %d\n",
               expected_len + sizeof(struct usb_packet_header), len);
        return;
    }
    
    /* For incomplete packets, we might need to buffer them */
    if (expected_len + sizeof(struct usb_packet_header) < len)
    {
        pr_warn("esp_usb: Received more data than expected: expected %lu, got %d\n",
               expected_len + sizeof(struct usb_packet_header), len);
        /* Continue processing with expected length */
    }
    
    /* Calculate and verify CRC16 */
    crc16_calculated = crc16_ccitt(data + sizeof(struct usb_packet_header),
                                   expected_len, 0x0000);
    if (crc16_calculated != crc16_received)
    {
        pr_err("esp_usb: CRC16 mismatch: received 0x%04x, calculated 0x%04x\n",
               crc16_received, crc16_calculated);
        return;
    }
    
    /* Allocate SKB for payload */
    rx_skb = esp_alloc_skb(expected_len);
    if (!rx_skb)
    {
        pr_err("esp_usb: Failed to allocate RX SKB\n");
        return;
    }
    
    /* Copy payload data */
    memcpy(skb_put(rx_skb, expected_len), 
           data + sizeof(struct usb_packet_header), expected_len);
    
    /* Parse ESP payload header to determine priority */
    if (expected_len >= sizeof(struct esp_payload_header))
    {
        payload_header = (struct esp_payload_header *)rx_skb->data;
        
        /* Determine priority queue based on interface type */
        if (payload_header->if_type == ESP_INTERNAL_IF)
        {
            priority_queue = PRIO_Q_HIGH;
        }
        else if (payload_header->if_type == ESP_HCI_IF)
        {
            priority_queue = PRIO_Q_MID;
        }
        else
        {
            priority_queue = PRIO_Q_LOW;
        }
        
        /* Enqueue to appropriate priority queue */
        skb_queue_tail(&usb_context.rx_q[priority_queue], rx_skb);
        
        /* Indicate reception of new packet */
        esp_process_new_packet_intr(usb_context.adapter);
        
        pr_info("esp_usb: Received packet of size %u, type %u, priority %u\n",
                expected_len, payload_header->if_type, priority_queue);
    }
    else
    {
        pr_err("esp_usb: Payload too small for ESP header\n");
        dev_kfree_skb(rx_skb);
    }
}

static void open_data_path(void)
{
    atomic_set(&tx_pending, 0);
    msleep(200);
    data_path = OPEN_DATAPATH;

    /* Async USB receive is already started in probe */
    pr_info("esp_usb: Data path opened, async receive already active\n");
}

static void close_data_path(void)
{
    data_path = CLOSE_DATAPATH;
    
    /* Stop async USB receive */
    if (usb_context.rx_urb)
    {
        usb_kill_urb(usb_context.rx_urb);
    }
    
    msleep(200);
}

int generate_slave_intr(void *context, u8 data)
{
    return 0;
}

int esp_validate_chipset(struct esp_adapter *adapter, u8 chipset)
{
    if (chipset == ESP_FIRMWARE_CHIP_ESP32S3)
    {
        adapter->chipset = chipset;
        esp_info("Chipset=%s ID=%02x detected over USB\n", esp_chipname_from_id(chipset), chipset);
    }
    return 0;
}

int esp_deinit_module(struct esp_adapter *adapter)
{
    return 0;
}

int esp_adjust_spi_clock(struct esp_adapter *adapter, u8 spi_clk_mhz)
{
    return 0;
}

static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb)
{
    u32 max_pkt_size = ESP_RX_BUFFER_SIZE - sizeof(struct esp_payload_header) - sizeof(struct usb_packet_header);
    struct esp_payload_header *payload_header = (struct esp_payload_header *)skb->data;
    struct esp_skb_cb *cb = NULL;

    if (!adapter || !adapter->if_context || !skb || !skb->data || !skb->len)
    {
        pr_err("esp_usb: Invalid args\n");
        if (skb)
        {
            dev_kfree_skb(skb);
            skb = NULL;
        }

        return -EINVAL;
    }

    if (skb->len > max_pkt_size)
    {
        pr_err("esp_usb: Drop pkt of len[%u] > max USB transport len[%u]\n",
               skb->len, max_pkt_size);
        dev_kfree_skb(skb);
        skb = NULL;
        return -EPERM;
    }

    if (!data_path)
    {
        pr_err("esp_usb: %u datapath closed\n", __LINE__);
        dev_kfree_skb(skb);
        return -EPERM;
    }

    cb = (struct esp_skb_cb *)skb->cb;
    if (cb && cb->priv && (atomic_read(&tx_pending) >= TX_MAX_PENDING_COUNT))
    {
        esp_tx_pause(cb->priv);
        dev_kfree_skb(skb);
        skb = NULL;

        if(usb_context.usb_workqueue) {
            queue_work(usb_context.usb_workqueue, &usb_context.usb_tx_work);
        }

        return -EBUSY;
    }

    /* Enqueue SKB in tx_q */
    if (payload_header->if_type == ESP_INTERNAL_IF)
    {
        skb_queue_tail(&usb_context.tx_q[PRIO_Q_HIGH], skb);
    }
    else if (payload_header->if_type == ESP_HCI_IF)
    {
        skb_queue_tail(&usb_context.tx_q[PRIO_Q_MID], skb);
    }
    else
    {
        skb_queue_tail(&usb_context.tx_q[PRIO_Q_LOW], skb);
        atomic_inc(&tx_pending);
    }

    if (usb_context.usb_workqueue)
        queue_work(usb_context.usb_workqueue, &usb_context.usb_tx_work);

    return 0;
}

static struct sk_buff *read_packet(struct esp_adapter *adapter)
{
    struct esp_usb_context *context;
    struct sk_buff *skb = NULL;

    if (!data_path)
    {
        return NULL;
    }

    if (!adapter || !adapter->if_context)
    {
        pr_err("esp_usb: Invalid args\n");
        return NULL;
    }

    context = adapter->if_context;

    if (context->udev)
    {
        skb = skb_dequeue(&(context->rx_q[PRIO_Q_HIGH]));
        if (!skb)
            skb = skb_dequeue(&(context->rx_q[PRIO_Q_MID]));
        if (!skb)
            skb = skb_dequeue(&(context->rx_q[PRIO_Q_LOW]));
    }
    else
    {
        pr_err("esp_usb: Invalid args\n");
        return NULL;
    }

    return skb;
}

// 新增：发送完成回调
static void esp_usb_tx_complete(struct urb *urb)
{
    struct sk_buff *usb_skb = urb->context;
    if (urb->status) {
        pr_err("esp_usb: TX URB error: %d\n", urb->status);
    }
    dev_kfree_skb(usb_skb);
    usb_free_urb(urb);
    // 继续调度下一个包
    if (usb_context.usb_workqueue)
        queue_work(usb_context.usb_workqueue, &usb_context.usb_tx_work);
}

static void esp_usb_tx_work(struct work_struct *work)
{
    struct sk_buff *tx_skb = NULL;
    struct esp_skb_cb *cb = NULL;
    struct usb_packet_header *header;
    struct sk_buff *usb_skb = NULL;
    u8 *usb_data;
    struct urb *tx_urb = NULL;

    mutex_lock(&usb_lock);

    /* Dequeue SKB from tx_q in priority order */
    tx_skb = skb_dequeue(&usb_context.tx_q[PRIO_Q_HIGH]);
    if (!tx_skb)
        tx_skb = skb_dequeue(&usb_context.tx_q[PRIO_Q_MID]);
    if (!tx_skb)
        tx_skb = skb_dequeue(&usb_context.tx_q[PRIO_Q_LOW]);

    if (tx_skb)
    {
        if (atomic_read(&tx_pending))
            atomic_dec(&tx_pending);

        /* Resume network tx queue if bearable load */
        cb = (struct esp_skb_cb *)tx_skb->cb;
        if (cb && cb->priv && atomic_read(&tx_pending) < TX_RESUME_THRESHOLD)
        {
            esp_tx_resume(cb->priv);
        }

        /* Allocate USB packet with header */
        usb_skb = esp_alloc_skb(tx_skb->len + sizeof(struct usb_packet_header));
        if (!usb_skb)
        {
            pr_err("esp_usb: Failed to allocate USB packet\n");
            dev_kfree_skb(tx_skb);
            mutex_unlock(&usb_lock);
            return;
        }

        /* Setup USB packet header */
        usb_data = skb_put(usb_skb, tx_skb->len + sizeof(struct usb_packet_header));
        header = (struct usb_packet_header *)usb_data;
        header->total_len = cpu_to_le16(tx_skb->len);  // 包含ESP payload header
        memcpy(usb_data + sizeof(struct usb_packet_header), tx_skb->data, tx_skb->len);
        header->crc16 = cpu_to_le16(crc16_ccitt(tx_skb->data, tx_skb->len, 0x0000));

        pr_info("esp_usb: Sending USB packet: total_len=%u, data_len=%u, crc16=0x%04x\n",
                usb_skb->len, tx_skb->len, le16_to_cpu(header->crc16));

        // 新增：分配URB并异步发送
        tx_urb = usb_alloc_urb(0, GFP_ATOMIC);
        if (!tx_urb) {
            pr_err("esp_usb: Failed to allocate TX URB\n");
            dev_kfree_skb(usb_skb);
            dev_kfree_skb(tx_skb);
            mutex_unlock(&usb_lock);
            return;
        }
        usb_fill_bulk_urb(tx_urb, usb_context.udev, usb_context.out_pipe,
                          usb_skb->data, usb_skb->len, esp_usb_tx_complete, usb_skb);
        // 这里不再立即释放usb_skb，回调里释放
        if (usb_submit_urb(tx_urb, GFP_ATOMIC)) {
            pr_err("esp_usb: Failed to submit TX URB\n");
            dev_kfree_skb(usb_skb);
            dev_kfree_skb(tx_skb);
            usb_free_urb(tx_urb);
            mutex_unlock(&usb_lock);
            return;
        }
        // tx_skb 立即释放
        dev_kfree_skb(tx_skb);
    }

    pr_info("esp_usb: tx write work done\n");

    mutex_unlock(&usb_lock);
}

static void esp_usb_rx_work(struct work_struct *work)
{
    /* This function is now used for retrying URB submission on errors */
    if (usb_context.is_alive && data_path)
    {
        esp_usb_submit_rx_urb();
    }
}

static int esp_usb_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    pr_info("esp_usb: USB device detected\n");
    struct usb_host_interface *iface_desc = NULL;
    struct usb_endpoint_descriptor *endpoint = NULL;
    
    usb_context.udev = interface_to_usbdev(interface);
    usb_context.usb_workqueue = create_workqueue("ESP_USB_WORK_QUEUE");
    INIT_WORK(&usb_context.usb_tx_work, esp_usb_tx_work);
    INIT_WORK(&usb_context.usb_rx_work, esp_usb_rx_work);
    
    /* Initialize async USB receive */
    usb_context.is_alive = 1;
    usb_context.rx_buffer_size = ESP_RX_BUFFER_SIZE;
    mutex_init(&usb_context.rx_lock);
    
    /* Allocate RX buffer */
    usb_context.rx_buffer = kmalloc(usb_context.rx_buffer_size, GFP_KERNEL);
    if (!usb_context.rx_buffer)
    {
        pr_err("esp_usb: Failed to allocate RX buffer\n");
        return -ENOMEM;
    }
    
    /* Allocate RX URB */
    usb_context.rx_urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!usb_context.rx_urb)
    {
        pr_err("esp_usb: Failed to allocate RX URB\n");
        kfree(usb_context.rx_buffer);
        return -ENOMEM;
    }
    
    for (int i = 0; i < MAX_PRIORITY_QUEUES; i++)
    {
        skb_queue_head_init(&usb_context.tx_q[i]);
        skb_queue_head_init(&usb_context.rx_q[i]);
    }

    // find input and output endpoint
    iface_desc = interface->cur_altsetting;
    for (int i = 0; i < iface_desc->desc.bNumEndpoints; i++)
    {
        endpoint = &iface_desc->endpoint[i].desc;

        // Vendor endpoints are typically bulk endpoints with specific addresses
        if (usb_endpoint_is_bulk_out(endpoint))
        {
            pr_info("Found bulk OUT endpoint: 0x%02x\n", endpoint->bEndpointAddress);
            usb_context.out_pipe = usb_sndbulkpipe(usb_context.udev, endpoint->bEndpointAddress);
        }

        if (usb_endpoint_is_bulk_in(endpoint))
        {
            pr_info("Found bulk IN endpoint: 0x%02x\n", endpoint->bEndpointAddress);
            usb_context.in_pipe = usb_rcvbulkpipe(usb_context.udev, endpoint->bEndpointAddress);
        }
    }

    /* Start async USB receive immediately after finding endpoints */
    esp_usb_submit_rx_urb();

    open_data_path();

    return 0;
}

static void esp_usb_disconnect(struct usb_interface *interface)
{
    pr_info("esp_usb: USB device disconnected\n");
    
    /* Stop async USB receive */
    usb_context.is_alive = 0;
    close_data_path();

    // 清空并释放所有 tx/rx 队列中的 skb
    for (int i = 0; i < MAX_PRIORITY_QUEUES; i++) {
        skb_queue_purge(&usb_context.tx_q[i]);
        skb_queue_purge(&usb_context.rx_q[i]);
    }

    // 销毁工作队列
    if (usb_context.usb_workqueue) {
        flush_workqueue(usb_context.usb_workqueue);
        destroy_workqueue(usb_context.usb_workqueue);
        usb_context.usb_workqueue = NULL;
    }
    
    /* Free async USB receive resources */
    if (usb_context.rx_urb)
    {
        usb_free_urb(usb_context.rx_urb);
        usb_context.rx_urb = NULL;
    }
    
    if (usb_context.rx_buffer)
    {
        kfree(usb_context.rx_buffer);
        usb_context.rx_buffer = NULL;
    }

    // 置空 udev 指针
    usb_context.udev = NULL;
}

int esp_init_interface_layer(struct esp_adapter *adapter, u32 speed)
{
    pr_info("esp_usb: esp_init_interface_layer\n");
    if (!adapter)
        return -EINVAL;

    adapter->if_context = &usb_context;
    adapter->if_ops = &if_ops;
    usb_context.adapter = adapter;

    return usb_register(&esp_usb_driver);
}

void esp_deinit_interface_layer(void)
{
    usb_deregister(&esp_usb_driver);
    pr_info("esp_usb: esp_deinit_interface_layer\n");
}