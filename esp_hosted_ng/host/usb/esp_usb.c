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
#include <linux/dma-mapping.h>
#include "esp_if.h"
#include "esp_api.h"
#include "esp_usb.h"
#include "esp_utils.h"
#include "esp_kernel_port.h"

#define TX_MAX_PENDING_COUNT 200
#define TX_RESUME_THRESHOLD (TX_MAX_PENDING_COUNT / 5)
#define USB_TX_TIMEOUT_MS 100 // 减少超时时间到1秒
#define USB_TX_RETRY_COUNT 3  // 发送重试次数
#define USB_EP_OUT_SIZE 64
#define MIN(a, b) ((a) < (b) ? (a) : (b))

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
static void usb_rx_routine(struct work_struct *work);
void push_urb_data(struct esp_usb_context *context, u8 *data, u32 len);

// USB性能优化相关函数
static int esp_usb_tx_with_retry(struct esp_usb_context *context, u8 *data, u32 len);

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

    while (retry_count < USB_TX_RETRY_COUNT)
    {
        ret = usb_bulk_msg(context->udev, context->out_pipe,
                           data, len, &actual_len, USB_TX_TIMEOUT_MS);

        if (ret == 0 && actual_len == len)
        {
            atomic_inc(&usb_tx_success_count);
            return 0; // 成功
        }

        retry_count++;
        if (retry_count < USB_TX_RETRY_COUNT)
        {
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

void push_urb_data(struct esp_usb_context *context, u8 *data, u32 len)
{
    static bool frame_active = false;
    static uint16_t received_total = 0;
    static uint16_t expected_total = 0;
    static uint8_t frame_buffer[ESP_RX_BUFFER_SIZE];
    struct esp_payload_header *header;
    struct sk_buff *skb = NULL;
    uint8_t prio = PRIO_Q_LOW;
    uint16_t computed_checksum;
    
    if (!context || !data || len < 2)
    {
        return;
    }
    
    // 检查是否有足够的数据来解析header
    if (!frame_active && len >= sizeof(struct esp_payload_header))
    {
        header = (struct esp_payload_header *)data;
        uint16_t payload_len = le16_to_cpu(header->len);
        uint16_t offset = le16_to_cpu(header->offset);
        
        expected_total = offset + payload_len;
        
        esp_dbg("USB RX: Header - payload_len=%d, offset=%d, expected_total=%d\n", 
                payload_len, offset, expected_total);
        
        if (expected_total > ESP_RX_BUFFER_SIZE)
        {
            esp_err("USB RX: Payload too large: %d > %d, actual_len: %d\n", expected_total, ESP_RX_BUFFER_SIZE, len);
            frame_active = false;
            return;
        }
        
        frame_active = true;
        received_total = 0;
        
        // 复制第一个包的数据
        int data_len = MIN(len, expected_total);
        if (data_len > 0)
        {
            memcpy(frame_buffer, data, data_len);
            received_total = data_len;
        }
    } else if (frame_active)
    {
        // 拼接剩余包数据
        int remain = expected_total - received_total;
        int to_copy = MIN(len, remain);
        
        if (to_copy > 0)
        {
            memcpy(frame_buffer + received_total, data, to_copy);
            received_total += to_copy;
        }
    }
    
    // 验证数据的完整性
    if (frame_active && received_total >= expected_total)
    {
        esp_dbg("USB RX: Frame received (%d bytes)\n", expected_total);
        
        // CRC校验
        header = (struct esp_payload_header *)frame_buffer;
        
        // 临时保存并清零checksum字段
        uint16_t original_checksum = le16_to_cpu(header->checksum);
        header->checksum = 0;
        
        // 计算整个数据包的checksum
        computed_checksum = compute_checksum(frame_buffer, expected_total);
        
        // 恢复原始checksum值
        header->checksum = cpu_to_le16(original_checksum);
        
        if (computed_checksum != original_checksum)
        {
            esp_err("USB RX: Checksum mismatch: computed=0x%04x, received=0x%04x\n", 
                    computed_checksum, original_checksum);
            frame_active = false;
            received_total = 0;
            expected_total = 0;
            return;
        }
        
        // 分配SKB
        skb = esp_alloc_skb(expected_total);
        if (!skb)
        {
            esp_err("USB RX:Failed to allocate SKB\n");
            frame_active = false;
            received_total = 0;
            expected_total = 0;
            return;
        }
        
        // 复制数据到SKB
        skb_put(skb, expected_total);
        memcpy(skb->data, frame_buffer, expected_total);
        
        // 解析header确定优先级
        header = (struct esp_payload_header *)skb->data;
        if (header->if_type == ESP_INTERNAL_IF)
        {
            prio = PRIO_Q_HIGH;
        }
        else if (header->if_type == ESP_HCI_IF)
        {
            prio = PRIO_Q_MID;
        }
        else
        {
            prio = PRIO_Q_LOW;
        }
        
        // 放入接收队列
        skb_queue_tail(&context->rx_q[prio], skb);
        
        esp_dbg("USB RX: Frame queued, type=%d, len=%d, prio=%d\n", 
                header->if_type, expected_total, prio);
        
        // 更新统计
        atomic_add(expected_total, &usb_rx_bytes_count);
        atomic_inc(&usb_rx_packet_count);
        
        // 重置状态
        frame_active = false;
        received_total = 0;
        expected_total = 0;
        
        // 触发上层处理
        esp_process_new_packet_intr(context->adapter);
    }
}

static void esp_usb_rx_complete(struct urb *urb)
{
    struct esp_usb_context *context = urb->context;

    if (!context)
    {
        esp_err("USB RX complete: invalid context\n");
        return;
    }

    switch (urb->status)
    {
    case 0:
        if (urb->actual_length > 2)
        {
            push_urb_data(context, context->rx_buffer, urb->actual_length);
        }
        break;
    case -EPIPE:
        context->usb_rx_pipe_status = -EPIPE;
        break;
    default:
        context->rx_urb_failed_count++;
    }

    if (context->rx_urb_failed_count < 3)
    {
        context->usb_rx_pipe_status = urb->status;
        if (!schedule_work(&context->rx_work))
        {
            esp_err("Failed to schedule USB RX work\n");
        }
    }
    else
    {
        context->usb_rx_pipe_status = -EPIPE;
        if (!schedule_work(&context->rx_work))
        {
            esp_err("Failed to schedule USB RX work\n");
        }
    }
}

static void usb_rx_routine(struct work_struct *work)
{
    struct esp_usb_context *context = container_of(work, struct esp_usb_context, rx_work);

    if (context->usb_rx_pipe_status == -EPIPE)
    {
        esp_err("USB RX pipe status error\n");
        context->usb_rx_pipe_status = 0;
        context->rx_urb_failed_count = 0;
        msleep(50);
    }

    if (context->usb_rx_pipe_status < 0)
    {
        msleep(20);
    }

    esp_usb_start_rx(context);
}

static int esp_usb_start_rx(struct esp_usb_context *context)
{
    int ret;

    if (!context || !context->udev || !context->rx_buffer)
    {
        esp_err("Invalid context for USB RX start\n");
        return -EINVAL;
    }

    usb_fill_bulk_urb(context->rx_urb, context->udev, context->in_pipe,
                      context->rx_buffer, ESP_RX_BUFFER_SIZE,
                      esp_usb_rx_complete, context);

    ret = usb_submit_urb(context->rx_urb, GFP_KERNEL);
    if (ret)
    {
        if (ret == -EPIPE)
        {
            usb_clear_halt(context->udev, context->in_pipe);
        }

        ++context->rx_urb_failed_count;
    }
    return 0;
}

static void esp_usb_stop_rx(struct esp_usb_context *context)
{
    if (!context)
        return;

    // 停止单个URB
    if (context->rx_urb)
    {
        usb_kill_urb(context->rx_urb);
        usb_free_urb(context->rx_urb);
        context->rx_urb = NULL;
    }

    // 释放接收缓冲区
    if (context->rx_buffer)
    {
        kfree(context->rx_buffer);
        context->rx_buffer = NULL;
    }

    // 清空接收队列
    for (int i = 0; i < MAX_PRIORITY_QUEUES; i++)
    {
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

    if (!adapter || !adapter->if_context)
    {
        esp_err("Invalid args for USB read_packet\n");
        return NULL;
    }

    context = adapter->if_context;

    if (!context)
    {
        esp_err("Invalid USB context\n");
        return NULL;
    }

    // 按优先级从队列中取出数据（非阻塞）
    skb = skb_dequeue(&context->rx_q[PRIO_Q_HIGH]);
    if (!skb)
        skb = skb_dequeue(&context->rx_q[PRIO_Q_MID]);
    if (!skb)
        skb = skb_dequeue(&context->rx_q[PRIO_Q_LOW]);

    if (skb)
    {
        print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 16, 1, skb->data, skb->len, false);
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
    u32 data_left;

    context = adapter->if_context;

    while (!kthread_should_stop())
    {

        if (atomic_read(&context->adapter->state) < ESP_CONTEXT_READY)
        {
            msleep(1);
            esp_dbg("not ready\n");
            continue;
        }

        if (atomic_read(&queue_items[PRIO_Q_HIGH]) > 0)
        {
            tx_skb = skb_dequeue(&(context->tx_q[PRIO_Q_HIGH]));
            if (!tx_skb)
            {
                continue;
            }
            atomic_dec(&queue_items[PRIO_Q_HIGH]);
        }
        else if (atomic_read(&queue_items[PRIO_Q_MID]) > 0)
        {
            tx_skb = skb_dequeue(&(context->tx_q[PRIO_Q_MID]));
            if (!tx_skb)
            {
                continue;
            }
            atomic_dec(&queue_items[PRIO_Q_MID]);
        }
        else if (atomic_read(&queue_items[PRIO_Q_LOW]) > 0)
        {
            tx_skb = skb_dequeue(&(context->tx_q[PRIO_Q_LOW]));
            if (!tx_skb)
            {
                continue;
            }
            atomic_dec(&queue_items[PRIO_Q_LOW]);
        }
        else
        {
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
        if (cb && cb->priv && atomic_read(&tx_pending) < TX_RESUME_THRESHOLD)
        {
            esp_tx_resume(cb->priv);
        }

        /* USB优化传输，使用重试机制 */
        pos = tx_skb->data;
        data_left = tx_skb->len;

        /* 使用优化的USB发送函数 */
        ret = esp_usb_tx_with_retry(context, pos, data_left);
        if (ret)
        {
            esp_err("USB optimized transfer failed: %d\n", ret);
            dev_kfree_skb(tx_skb);
            continue;
        }

        dev_kfree_skb(tx_skb);
        tx_skb = NULL;
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

    usb_context.rx_urb_failed_count = 0;

    usb_context.rx_urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!usb_context.rx_urb)
    {
        esp_err("Failed to allocate USB RX URB\n");
        return -ENOMEM;
    }

    // 先设置 udev，然后才能用于DMA分配
    usb_context.udev = interface_to_usbdev(interface);
    atomic_set(&tx_pending, 0);
    usb_context.adapter = esp_get_adapter();

    // 分配接收缓冲区 - 使用 kmalloc 而不是 dma_alloc_coherent
    usb_context.rx_buffer = kmalloc(ESP_RX_BUFFER_SIZE, GFP_KERNEL);
    if (!usb_context.rx_buffer)
    {
        esp_err("Failed to allocate USB RX buffer\n");
        usb_free_urb(usb_context.rx_urb);
        return -ENOMEM;
    }

    /* Initialize queues */
    for (i = 0; i < MAX_PRIORITY_QUEUES; i++)
    {
        skb_queue_head_init(&usb_context.tx_q[i]);
        atomic_set(&queue_items[i], 0);
    }

    // 初始化接收队列
    for (i = 0; i < MAX_PRIORITY_QUEUES; i++)
    {
        skb_queue_head_init(&usb_context.rx_q[i]);
    }


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

    // 初始化USB接收工作队列
    INIT_WORK(&usb_context.rx_work, usb_rx_routine);

    // 启动USB异步接收
    if (esp_usb_start_rx(&usb_context))
    {
        esp_err("Failed to start USB RX\n");
        kfree(usb_context.rx_buffer);
        usb_free_urb(usb_context.rx_urb);
        return -ENOMEM;
    }

    tx_thread = kthread_run(tx_process, usb_context.adapter, "esp_usb_tx_thread");

    atomic_set(&usb_context.adapter->state, ESP_CONTEXT_READY);
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
    if (tx_thread)
    {
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
	adapter->if_type = ESP_IF_TYPE_SPI;
    usb_context.adapter = adapter;

    return usb_register(&esp_usb_driver);
}

void esp_deinit_interface_layer(void)
{
    usb_deregister(&esp_usb_driver);
    pr_info("esp_usb: esp_deinit_interface_layer\n");
}