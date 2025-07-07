#include "utils.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/crc16.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include "esp_if.h"
#include "esp_api.h"
#include "esp_usb.h"
#include "esp_utils.h"
#include "esp_kernel_port.h"

#define ESP_RX_BUFFER_SIZE 2048  // 增加到与SDIO一致的大小
#define TX_MAX_PENDING_COUNT 200
#define TX_RESUME_THRESHOLD (TX_MAX_PENDING_COUNT / 5)
#define USB_TX_TIMEOUT_MS 100   // 减少超时时间到1秒
#define USB_TX_RETRY_COUNT 3     // 发送重试次数

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
static int tx_process(void *data);

// USB异步接收相关函数声明
static void esp_usb_rx_complete(struct urb *urb);
static int esp_usb_start_rx(struct esp_usb_context *context);
static void esp_usb_stop_rx(struct esp_usb_context *context);
static struct sk_buff *process_received_usb_data(struct esp_usb_context *context, u8 *data, int len);

// USB性能优化相关函数
static int esp_usb_tx_with_retry(struct esp_usb_context *context, u8 *data, u32 len);
static void esp_usb_performance_monitor(struct esp_usb_context *context);

struct task_struct *tx_thread;

volatile u8 host_sleep;
volatile u8 data_path;
struct esp_usb_context usb_context;
static atomic_t tx_pending;
static atomic_t queue_items[MAX_PRIORITY_QUEUES];

// 性能统计
static atomic_t usb_tx_success_count;
static atomic_t usb_tx_fail_count;
static atomic_t usb_rx_packet_count;
static atomic_t usb_rx_bytes_count;

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

static void open_data_path(void)
{
    atomic_set(&tx_pending, 0);
    // 重置性能统计
    atomic_set(&usb_tx_success_count, 0);
    atomic_set(&usb_tx_fail_count, 0);
    atomic_set(&usb_rx_packet_count, 0);
    atomic_set(&usb_rx_bytes_count, 0);
    msleep(200);
    data_path = OPEN_DATAPATH;
}

static void close_data_path(void)
{
    data_path = CLOSE_DATAPATH;
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

// USB发送重试机制
static int esp_usb_tx_with_retry(struct esp_usb_context *context, u8 *data, u32 len)
{
    int ret, retry_count = 0;
    int actual_len = 0;
    
    while (retry_count < USB_TX_RETRY_COUNT) {
        ret = usb_bulk_msg(context->udev, context->out_pipe,
                           data, len, &actual_len, USB_TX_TIMEOUT_MS);
        
        if (ret == 0 && actual_len == len) {
            atomic_inc(&usb_tx_success_count);
            return 0; // 成功
        }
        
        retry_count++;
        if (retry_count < USB_TX_RETRY_COUNT) {
            esp_warn("USB TX retry %d/%d: ret=%d, actual=%d, expected=%d\n", 
                     retry_count, USB_TX_RETRY_COUNT, ret, actual_len, len);
            usleep_range(100, 500); // 短暂延迟后重试
        }
    }
    
    atomic_inc(&usb_tx_fail_count);
    esp_err("USB TX failed after %d retries: ret=%d, actual=%d, expected=%d\n", 
            retry_count, ret, actual_len, len);
    return -EIO;
}

// 性能监控
static void esp_usb_performance_monitor(struct esp_usb_context *context)
{
    static unsigned long last_print_time = 0;
    unsigned long current_time = jiffies;
    
    // 每5秒打印一次性能统计
    if (time_after(current_time, last_print_time + 5 * HZ)) {
        int tx_success = atomic_read(&usb_tx_success_count);
        int tx_fail = atomic_read(&usb_tx_fail_count);
        int rx_packets = atomic_read(&usb_rx_packet_count);
        int rx_bytes = atomic_read(&usb_rx_bytes_count);
        
        esp_info("USB Performance: TX success=%d fail=%d, RX packets=%d bytes=%d\n",
                 tx_success, tx_fail, rx_packets, rx_bytes);
        
        last_print_time = current_time;
    }
}

// USB异步接收实现
static void esp_usb_rx_complete(struct urb *urb)
{
    struct esp_usb_context *context = urb->context;
    struct sk_buff *skb = NULL;
    unsigned long flags;
    
    if (!context) {
        esp_err("USB RX complete: invalid context\n");
        return;
    }
    
    if (urb->status == 0 && urb->actual_length > 0) {
        // 更新接收统计
        atomic_add(urb->actual_length, &usb_rx_bytes_count);
        atomic_inc(&usb_rx_packet_count);
        
        // 处理接收到的数据
        skb = process_received_usb_data(context, urb->transfer_buffer, urb->actual_length);
        if (skb) {
            // 触发上层处理，类似SDIO的中断处理
            esp_process_new_packet_intr(context->adapter);
        }
    } else if (urb->status != -ENOENT && urb->status != -ECONNRESET) {
        esp_err("USB RX error: %d\n", urb->status);
    }
    
    // 重新提交URB继续接收
    spin_lock_irqsave(&context->rx_lock, flags);
    if (atomic_read(&context->rx_active)) {
        urb->dev = context->udev;
        urb->pipe = context->in_pipe;
        urb->transfer_buffer = urb->transfer_buffer;
        urb->transfer_buffer_length = ESP_RX_BUFFER_SIZE;
        urb->complete = esp_usb_rx_complete;
        urb->context = context;
        
        if (usb_submit_urb(urb, GFP_ATOMIC)) {
            esp_err("Failed to resubmit USB RX URB\n");
        }
    }
    spin_unlock_irqrestore(&context->rx_lock, flags);
}

static struct sk_buff *process_received_usb_data(struct esp_usb_context *context, u8 *data, int len)
{
    struct sk_buff *skb = NULL;
    struct esp_payload_header *header = NULL;
    u16 payload_len, offset;
    uint8_t prio = PRIO_Q_LOW;
    
    if (!data || len < sizeof(struct esp_payload_header)) {
        esp_err("Invalid USB RX data: len=%d\n", len);
        return NULL;
    }
    
    header = (struct esp_payload_header *)data;
    payload_len = le16_to_cpu(header->len);
    offset = le16_to_cpu(header->offset);
    
    esp_dbg("USB RX: received %d bytes, header: payload_len=%d, offset=%d\n", 
            len, payload_len, offset);
    
    // 增加更严格的长度验证
    if (payload_len == 0) {
        esp_dbg("USB RX: empty packet, ignoring\n");
        return NULL;
    }
    
    // 检查payload_len是否合理（不应该超过ESP_RX_BUFFER_SIZE）
    if (payload_len > ESP_RX_BUFFER_SIZE) {
        esp_err("USB RX: payload_len too large: %d > %d\n", payload_len, ESP_RX_BUFFER_SIZE);
        return NULL;
    }
    
    // 验证数据长度 - 更宽松的检查
    if (payload_len + offset > len) {
        esp_warn("USB RX data truncated: expected=%d, actual=%d, payload_len=%d, offset=%d\n", 
                 payload_len + offset, len, payload_len, offset);
        
        // 如果数据不完整，尝试只处理可用的数据
        if (len > offset) {
            payload_len = len - offset;
            esp_info("USB RX: adjusting payload_len to %d\n", payload_len);
        } else {
            esp_err("USB RX: insufficient data even for header\n");
            return NULL;
        }
    }
    
    // 分配SKB
    skb = esp_alloc_skb(payload_len + offset);
    if (!skb) {
        esp_err("Failed to allocate SKB for USB RX\n");
        return NULL;
    }
    
    // 复制数据
    skb_put(skb, payload_len + offset);
    memcpy(skb->data, data, payload_len + offset);
    
    // 根据接口类型确定优先级
    if (header->if_type == ESP_INTERNAL_IF) {
        prio = PRIO_Q_HIGH;
    } else if (header->if_type == ESP_HCI_IF) {
        prio = PRIO_Q_MID;
    } else {
        prio = PRIO_Q_LOW;
    }
    
    // 放入接收队列
    skb_queue_tail(&context->rx_q[prio], skb);
    
    esp_dbg("USB RX: type=%d, len=%d, prio=%d\n", header->if_type, payload_len, prio);
    
    return skb;
}

static int esp_usb_start_rx(struct esp_usb_context *context)
{
    int i, ret;
    unsigned long flags;
    
    if (!context || !context->udev) {
        esp_err("Invalid context for USB RX start\n");
        return -EINVAL;
    }
    
    spin_lock_irqsave(&context->rx_lock, flags);
    atomic_set(&context->rx_active, 1);
    spin_unlock_irqrestore(&context->rx_lock, flags);
    
    // 初始化接收队列
    for (i = 0; i < MAX_PRIORITY_QUEUES; i++) {
        skb_queue_head_init(&context->rx_q[i]);
    }
    
    // 分配并提交URB
    for (i = 0; i < USB_RX_URB_COUNT; i++) {
        context->rx_buffer[i] = kmalloc(ESP_RX_BUFFER_SIZE, GFP_KERNEL);
        if (!context->rx_buffer[i]) {
            esp_err("Failed to allocate USB RX buffer %d\n", i);
            goto cleanup;
        }
        
        context->rx_urb[i] = usb_alloc_urb(0, GFP_KERNEL);
        if (!context->rx_urb[i]) {
            esp_err("Failed to allocate USB RX URB %d\n", i);
            kfree(context->rx_buffer[i]);
            context->rx_buffer[i] = NULL;
            goto cleanup;
        }
        
        usb_fill_bulk_urb(context->rx_urb[i], context->udev, context->in_pipe,
                          context->rx_buffer[i], ESP_RX_BUFFER_SIZE,
                          esp_usb_rx_complete, context);
        
        ret = usb_submit_urb(context->rx_urb[i], GFP_KERNEL);
        if (ret) {
            esp_err("Failed to submit USB RX URB %d: %d\n", i, ret);
            usb_free_urb(context->rx_urb[i]);
            kfree(context->rx_buffer[i]);
            context->rx_urb[i] = NULL;
            context->rx_buffer[i] = NULL;
            goto cleanup;
        }
    }
    
    esp_info("USB RX started with %d URBs\n", USB_RX_URB_COUNT);
    return 0;
    
cleanup:
    esp_usb_stop_rx(context);
    return -ENOMEM;
}

static void esp_usb_stop_rx(struct esp_usb_context *context)
{
    int i;
    unsigned long flags;
    
    if (!context)
        return;
    
    spin_lock_irqsave(&context->rx_lock, flags);
    atomic_set(&context->rx_active, 0);
    spin_unlock_irqrestore(&context->rx_lock, flags);
    
    // 停止所有URB
    for (i = 0; i < USB_RX_URB_COUNT; i++) {
        if (context->rx_urb[i]) {
            usb_kill_urb(context->rx_urb[i]);
            usb_free_urb(context->rx_urb[i]);
            context->rx_urb[i] = NULL;
        }
        if (context->rx_buffer[i]) {
            kfree(context->rx_buffer[i]);
            context->rx_buffer[i] = NULL;
        }
    }
    
    // 清空接收队列
    for (i = 0; i < MAX_PRIORITY_QUEUES; i++) {
        skb_queue_purge(&context->rx_q[i]);
    }
    
    esp_info("USB RX stopped\n");
}

static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb)
{
    u32 max_pkt_size = ESP_RX_BUFFER_SIZE - sizeof(struct esp_payload_header);
    struct esp_payload_header *payload_header = (struct esp_payload_header *)skb->data;
    struct esp_skb_cb *cb = NULL;
    uint8_t prio = PRIO_Q_LOW;

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

        return -EBUSY;
    }

    /* Enqueue SKB in tx_q */
    if (payload_header->if_type == ESP_INTERNAL_IF)
    {
        prio = PRIO_Q_HIGH;
    }
    else if (payload_header->if_type == ESP_HCI_IF)
    {
        prio = PRIO_Q_MID;
    }
    else
    {
        prio = PRIO_Q_LOW;
    }

    atomic_inc(&queue_items[prio]);
    skb_queue_tail(&(usb_context.tx_q[prio]), skb);

    return 0;
}

static struct sk_buff *read_packet(struct esp_adapter *adapter)
{
    struct esp_usb_context *context = NULL;
    struct sk_buff *skb = NULL;
    
    if (!adapter || !adapter->if_context) {
        esp_err("Invalid args for USB read_packet\n");
        return NULL;
    }
    
    context = adapter->if_context;
    
    if (!context) {
        esp_err("Invalid USB context\n");
        return NULL;
    }
    
    // 按优先级从队列中取出数据（非阻塞）
    skb = skb_dequeue(&context->rx_q[PRIO_Q_HIGH]);
    if (!skb)
        skb = skb_dequeue(&context->rx_q[PRIO_Q_MID]);
    if (!skb)
        skb = skb_dequeue(&context->rx_q[PRIO_Q_LOW]);
    
    if (skb) {
        esp_dbg("USB read_packet: got skb len=%d\n", skb->len);
    }
    
    return skb;
}

static int tx_process(void *data)
{
    int ret = 0;
    struct sk_buff *tx_skb = NULL;
    struct esp_adapter *adapter = (struct esp_adapter *)data;
    struct esp_usb_context *context = NULL;
    struct esp_skb_cb *cb = NULL;
    u8 *pos = NULL;
    u32 data_left, len_to_send;

    context = adapter->if_context;

    while (!kthread_should_stop()) {

        if (atomic_read(&context->adapter->state) < ESP_CONTEXT_READY) {
            msleep(1);
            esp_dbg("not ready\n");
            continue;
        }

        if (atomic_read(&queue_items[PRIO_Q_HIGH]) > 0) {
            tx_skb = skb_dequeue(&(context->tx_q[PRIO_Q_HIGH]));
            if (!tx_skb) {
                continue;
            }
            atomic_dec(&queue_items[PRIO_Q_HIGH]);
        } else if (atomic_read(&queue_items[PRIO_Q_MID]) > 0) {
            tx_skb = skb_dequeue(&(context->tx_q[PRIO_Q_MID]));
            if (!tx_skb) {
                continue;
            }
            atomic_dec(&queue_items[PRIO_Q_MID]);
        } else if (atomic_read(&queue_items[PRIO_Q_LOW]) > 0) {
            tx_skb = skb_dequeue(&(context->tx_q[PRIO_Q_LOW]));
            if (!tx_skb) {
                continue;
            }
            atomic_dec(&queue_items[PRIO_Q_LOW]);
        } else {
            /* esp_verbose("not ready high=%d mid=%d low=%d\n",
                    atomic_read(&queue_items[PRIO_Q_HIGH]),
                    atomic_read(&queue_items[PRIO_Q_MID]),
                    atomic_read(&queue_items[PRIO_Q_LOW])); */
            msleep(1);
            continue;
        }

        if (atomic_read(&tx_pending))
            atomic_dec(&tx_pending);

        /* resume network tx queue if bearable load */
        cb = (struct esp_skb_cb *)tx_skb->cb;
        if (cb && cb->priv && atomic_read(&tx_pending) < TX_RESUME_THRESHOLD) {
            esp_tx_resume(cb->priv);
        }

        /* USB优化传输，使用重试机制 */
        pos = tx_skb->data;
        data_left = tx_skb->len;

        /* 使用优化的USB发送函数 */
        ret = esp_usb_tx_with_retry(context, pos, data_left);
        if (ret) {
            esp_err("USB optimized transfer failed: %d\n", ret);
            dev_kfree_skb(tx_skb);
            continue;
        }

        dev_kfree_skb(tx_skb);
        tx_skb = NULL;
        
        // 性能监控
        esp_usb_performance_monitor(context);
    }

    do_exit(0);
    return 0;
}

static int esp_usb_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    struct usb_host_interface *iface_desc = NULL;
    struct usb_endpoint_descriptor *endpoint = NULL;
    int i;

    pr_info("esp_usb: USB device detected\n");

    // 初始化数据路径状态
    data_path = OPEN_DATAPATH;

    usb_context.udev = interface_to_usbdev(interface);
    atomic_set(&tx_pending, 0);
    usb_context.adapter = esp_get_adapter();

    /* Initialize queues */
    for (i = 0; i < MAX_PRIORITY_QUEUES; i++)
    {
        skb_queue_head_init(&usb_context.tx_q[i]);
        atomic_set(&queue_items[i], 0);
    }

    // 初始化USB异步接收相关
    spin_lock_init(&usb_context.rx_lock);
    atomic_set(&usb_context.rx_active, 0);

    // find input and output endpoint
    iface_desc = interface->cur_altsetting;
    for (i = 0; i < iface_desc->desc.bNumEndpoints; i++)
    {
        endpoint = &iface_desc->endpoint[i].desc;

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

    // 启动USB异步接收
    if (esp_usb_start_rx(&usb_context)) {
        esp_err("Failed to start USB RX\n");
        return -ENOMEM;
    }

    tx_thread = kthread_run(tx_process, usb_context.adapter, "esp_usb_tx_thread");

    atomic_set(&usb_context.adapter->state, ESP_CONTEXT_RX_READY);
    open_data_path();
    return 0;
}

static void esp_usb_disconnect(struct usb_interface *interface)
{
    int i;

    pr_info("esp_usb: USB device disconnected\n");

    close_data_path();

    // 停止USB异步接收
    esp_usb_stop_rx(&usb_context);

    // 停止传输线程
    if (tx_thread) {
        kthread_stop(tx_thread);
        tx_thread = NULL;
    }

    // 清空并释放所有 tx/rx 队列中的 skb
    for (i = 0; i < MAX_PRIORITY_QUEUES; i++)
    {
        skb_queue_purge(&usb_context.tx_q[i]);
        skb_queue_purge(&usb_context.rx_q[i]);
    }

    if (usb_context.adapter)
    {
        esp_remove_card(usb_context.adapter);
    }

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