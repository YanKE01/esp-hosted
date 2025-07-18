#include "sdkconfig.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <rom/rtc.h>
#include "esp.h"
#include "esp_log.h"
#include "interface.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "device/usbd.h"
#include "esp_private/usb_phy.h"
#include "usb_descriptors.h"
#include "endian.h"
#include "esp_fw_version.h"

#define MAX_PAYLOAD_SIZE CFG_TUD_VENDOR_RX_BUFSIZE
#define HEADER_SIZE 4
#define FRAME_QUEUE_SIZE 20  // 增加队列大小，容纳更多帧
#define USB_RX_BUF_NUM 30    // 增加缓冲区数量，匹配队列大小

static interface_context_t context;
static interface_handle_t if_handle_g;

// Add semaphore for USB TX completion
static SemaphoreHandle_t usb_tx_sem = NULL;

// 预分配的USB接收缓冲区
static uint8_t usb_rx_buffer[USB_RX_BUF_NUM][MAX_PAYLOAD_SIZE];
static bool usb_rx_buffer_used[USB_RX_BUF_NUM] = {false};
static uint32_t buffer_alloc_count = 0;
static uint32_t buffer_release_count = 0;
static uint32_t emergency_alloc_count = 0;

// 用于解析USB包
static uint8_t frame_buffer[MAX_PAYLOAD_SIZE] = {0};
static uint16_t received_total = 0;
static uint16_t expected_total = 0;
static bool frame_active = false;

// 队列缓冲区
typedef struct {
    uint8_t *data;
    uint16_t len;
} usb_frame_t;

static QueueHandle_t frame_queue;

static int32_t esp_usb_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle);
static int esp_usb_read(interface_handle_t *handle, interface_buffer_handle_t *buf_handle);
static void esp_usb_deinit(interface_handle_t *handle);
static interface_handle_t *esp_usb_init(void);
static void print_queue_status(void);

static const char TAG[] = "FW_USB";

// 获取可用的预分配缓冲区
static uint8_t* get_available_rx_buffer(void)
{
    for (int i = 0; i < USB_RX_BUF_NUM; i++) {
        if (!usb_rx_buffer_used[i]) {
            usb_rx_buffer_used[i] = true;
            buffer_alloc_count++;
            // ESP_LOGI(TAG, "Allocated RX buffer %d (total: %lu)", i, buffer_alloc_count);
            return usb_rx_buffer[i];
        }
    }

    // 如果所有缓冲区都被占用，强制释放第一个缓冲区
    ESP_LOGW(TAG, "All buffers in use, forcing release of buffer 0");
    usb_rx_buffer_used[0] = false;
    usb_rx_buffer_used[0] = true;
    buffer_alloc_count++;
    emergency_alloc_count++;
    // ESP_LOGI(TAG, "Emergency allocated RX buffer 0 (emergency: %lu)", emergency_alloc_count);
    return usb_rx_buffer[0];
}

// 释放预分配缓冲区
static void release_rx_buffer(void *buffer)
{
    for (int i = 0; i < USB_RX_BUF_NUM; i++) {
        if (buffer == usb_rx_buffer[i]) {
            usb_rx_buffer_used[i] = false;
            // buffer_release_count++;
            // ESP_LOGI(TAG, "Released RX buffer %d (total: %lu)", i, buffer_release_count);
            return;
        }
    }
    ESP_LOGE(TAG, "Attempted to release unknown buffer");
}

void tud_mount_cb(void)
{
    ESP_LOGI(TAG, "USB device mounted");
    // 触发ESP_OPEN_DATA_PATH事件
    if (context.event_handler) {
        context.event_handler(ESP_OPEN_DATA_PATH);
    }
}
void tud_umount_cb(void)
{
    ESP_LOGI(TAG, "USB device unmounted");
}
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    ESP_LOGI(TAG, "USB suspended");
}
void tud_resume_cb(void)
{
    ESP_LOGI(TAG, "USB resumed");
}

if_ops_t if_ops = {
    .write = esp_usb_write,
    .read = esp_usb_read,
    .init = esp_usb_init,
    .deinit = esp_usb_deinit,
};

interface_context_t *interface_insert_driver(int (*event_handler)(uint8_t val))
{
    ESP_LOGI(TAG, "Using SDIO interface");
    memset(&context, 0, sizeof(context));

    context.type = SDIO;
    context.if_ops = &if_ops;
    context.event_handler = event_handler;

    return &context;
}

int interface_remove_driver()
{
    memset(&context, 0, sizeof(context));
    return 0;
}

esp_err_t send_bootup_event_to_host(uint8_t cap)
{
    struct esp_payload_header *header = NULL;
    struct esp_internal_bootup_event *event = NULL;
    struct fw_data * fw_p = NULL;
    interface_buffer_handle_t buf_handle = {0};
    uint8_t * pos = NULL;
    esp_err_t ret = ESP_OK;
    uint16_t len = 0;

    memset(&buf_handle, 0, sizeof(buf_handle));

    buf_handle.payload = heap_caps_malloc(RX_BUF_SIZE, MALLOC_CAP_DMA);
    assert(buf_handle.payload);
    memset(buf_handle.payload, 0, RX_BUF_SIZE);

    header = (struct esp_payload_header *) buf_handle.payload;

    header->if_type = ESP_INTERNAL_IF;
    header->if_num = 0;
    header->offset = htole16(sizeof(struct esp_payload_header));

    event = (struct esp_internal_bootup_event*)(buf_handle.payload + sizeof(struct esp_payload_header));

    event->header.event_code = ESP_INTERNAL_BOOTUP_EVENT;
    event->header.status = 0;

    pos = event->data;

    /* TLVs start */

    /* TLV - Board type */
    *pos = ESP_BOOTUP_FIRMWARE_CHIP_ID;   pos++; len++;
    *pos = LENGTH_1_BYTE;                 pos++; len++;
    *pos = CONFIG_IDF_FIRMWARE_CHIP_ID;   pos++; len++;

    /* TLV - Capability */
    *pos = ESP_BOOTUP_CAPABILITY;         pos++; len++;
    *pos = LENGTH_1_BYTE;                 pos++; len++;
    *pos = cap;                           pos++; len++;

    /* TLV - FW data */
    *pos = ESP_BOOTUP_FW_DATA;            pos++; len++;
    *pos = sizeof(struct fw_data);        pos++; len++;
    fw_p = (struct fw_data *) pos;
    /* core0 sufficient now */
    ESP_LOGI(TAG, "last reset cause: %0xx", rtc_get_reset_reason(0));
    fw_p->last_reset_reason = htole32(rtc_get_reset_reason(0));
    memcpy(fw_p->version.project_name, PROJECT_NAME, strlen(PROJECT_NAME));
    fw_p->version.project_name[strlen(PROJECT_NAME)] = '\0';
    fw_p->version.major1 = PROJECT_VERSION_MAJOR_1;
    fw_p->version.major2 = PROJECT_VERSION_MAJOR_2;
    fw_p->version.minor  = PROJECT_VERSION_MINOR;
    fw_p->version.revision_patch_1  = PROJECT_REVISION_PATCH_1;
    fw_p->version.revision_patch_2  = PROJECT_REVISION_PATCH_2;
    pos += sizeof(struct fw_data);
    len += sizeof(struct fw_data);

    /* TLVs end */
    event->len = len;
    buf_handle.payload_len = len + sizeof(struct esp_internal_bootup_event) + sizeof(struct esp_payload_header);

    /* payload len = Event len + sizeof(event len) */
    len += 1;
    event->header.len = htole16(len);

    header->len = htole16(buf_handle.payload_len - sizeof(struct esp_payload_header));

    header->checksum = htole16(compute_checksum(buf_handle.payload, buf_handle.payload_len));

    ESP_LOGI(TAG, "Send bootup check: %x", header->checksum);

    // 构造USB包
    int32_t total_len = buf_handle.payload_len;

    if (total_len > MAX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Payload size too large, max:%" PRIu16 ", current:%" PRId32, MAX_PAYLOAD_SIZE, total_len);
        free(buf_handle.payload);
        return ESP_FAIL;
    }

    ret = tud_vendor_n_write(0, buf_handle.payload, total_len);

    if (ret == 0) {
        ESP_LOGE(TAG, "Failed to send USB packet, ret:%d", ret);
        free(buf_handle.payload);
        return ESP_FAIL;
    }

    tud_vendor_n_flush(0);

    // Wait for TX completion - will block until TX is done
    xSemaphoreTake(usb_tx_sem, portMAX_DELAY);

    free(buf_handle.payload);

    return ESP_OK;
}

// USB Driver Related Functions

static void usb_phy_init(void)
{
    usb_phy_handle_t phy_hdl;
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .target = USB_PHY_TARGET_INT
    };
    usb_new_phy(&phy_conf, &phy_hdl);
}

static void tusb_device_task(void *pvParameters)
{
    while (1) {
        tud_task();
    }
}

void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize)
{
    while (tud_vendor_n_available(itf)) {
        uint8_t temp_buf[bufsize];
        int read_len = tud_vendor_n_read(itf, temp_buf, sizeof(temp_buf));
        if (read_len <= 0) {
            return;
        }

        if (!frame_active && read_len >= sizeof(struct esp_payload_header)) {
            // 解析ESP payload header
            struct esp_payload_header *header = (struct esp_payload_header *)temp_buf;
            uint16_t payload_len = le16toh(header->len);
            uint16_t offset = le16toh(header->offset);

            expected_total = offset + payload_len;

            // ESP_LOGI(TAG, "Header: payload_len=%d, offset=%d, expected_total=%d", payload_len, offset, expected_total);

            if (expected_total > MAX_PAYLOAD_SIZE) {
                ESP_LOGE(TAG, "Payload too large");
                frame_active = false;
                return;
            }

            frame_active = true;
            received_total = 0;

            // 复制第一个包的数据
            int data_len = read_len;
            if (data_len > 0) {
                memcpy(frame_buffer, temp_buf, data_len);
                received_total = data_len;
            }
        } else if (frame_active) {
            // 拼接剩余包数据
            int remain = expected_total - received_total;
            int to_copy = (read_len > remain) ? remain : read_len;

            if (to_copy > 0) {
                memcpy(frame_buffer + received_total, temp_buf, to_copy);
                received_total += to_copy;
            }
        }

        // 验证数据的完整性
        if (frame_active && received_total >= expected_total) {
            // ESP_LOGI(TAG, "Frame received (%d bytes)", expected_total);

            // 使用预分配的缓冲区
            uint8_t *rx_buffer = get_available_rx_buffer();
            if (!rx_buffer) {
                // 如果缓冲区耗尽，强制释放第一个缓冲区并继续处理
                // ESP_LOGW(TAG, "Buffer exhaustion, forcing release of buffer 0");
                release_rx_buffer(usb_rx_buffer[0]);
                rx_buffer = get_available_rx_buffer();
                if (!rx_buffer) {
                    // ESP_LOGE(TAG, "Critical: Still no buffer available after force release");
                    frame_active = false;
                    received_total = 0;
                    expected_total = 0;
                    return;
                }
            }

            usb_frame_t frame;
            frame.data = rx_buffer;
            frame.len = expected_total;

            // 复制数据到预分配缓冲区
            memcpy(frame.data, frame_buffer, expected_total);

            // 使用阻塞方式入队，确保数据不丢失
            if (xQueueSend(frame_queue, &frame, portMAX_DELAY) != pdTRUE) {
                // 如果100ms内无法入队，说明队列已满，这是异常情况
                // ESP_LOGE(TAG, "Queue full timeout, dropping frame (this should not happen)");
                release_rx_buffer(frame.data);
            } else {
                // ESP_LOGI(TAG, "Frame queued successfully, len: %d", expected_total);

                // 打印当前队列状态
                print_queue_status();
            }

            // 重置状态
            frame_active = false;
            received_total = 0;
            expected_total = 0;
        }
    }
}

// 队列状态监控
static void print_queue_status(void)
{
    if (frame_queue) {
        int queue_count = uxQueueMessagesWaiting(frame_queue);
        int used_buffers = 0;
        for (int i = 0; i < USB_RX_BUF_NUM; i++) {
            if (usb_rx_buffer_used[i]) {
                used_buffers++;
            }
        }

        // ESP_LOGI(TAG, "Queue status: %d/%d frames, Buffers: %d/%d used",
        //          queue_count, FRAME_QUEUE_SIZE, used_buffers, USB_RX_BUF_NUM);

        if (queue_count > FRAME_QUEUE_SIZE * 2 / 3) {
            // ESP_LOGW(TAG, "High queue utilization: %d%%", (queue_count * 100) / FRAME_QUEUE_SIZE);
        }
    }
}

static interface_handle_t *esp_usb_init(void)
{
    usb_phy_init();
    if (!tusb_init()) {
        ESP_LOGE(TAG, "TinyUSB init failed");
        return NULL;
    }

    // Create TX semaphore
    usb_tx_sem = xSemaphoreCreateBinary();
    if (!usb_tx_sem) {
        ESP_LOGE(TAG, "Failed to create TX semaphore");
        return NULL;
    }
    // Initialize semaphore as taken
    xSemaphoreGive(usb_tx_sem);

    xTaskCreate(tusb_device_task, "tusb_device_task", 10 * 1024, NULL, 14, NULL);

    // 创建接收队列
    frame_queue = xQueueCreate(FRAME_QUEUE_SIZE, sizeof(usb_frame_t));
    if (!frame_queue) {
        ESP_LOGE(TAG, "Failed to create frame queue");
        return NULL;
    }

    // 初始化预分配缓冲区状态
    for (int i = 0; i < USB_RX_BUF_NUM; i++) {
        usb_rx_buffer_used[i] = false;
    }

    // 初始化帧接收状态
    frame_active = false;
    received_total = 0;
    expected_total = 0;

    memset(&if_handle_g, 0, sizeof(if_handle_g));
    if_handle_g.state = INIT;

    ESP_LOGI(TAG, "USB init completed - Queue: %d frames, Buffers: %d, Max payload: %d bytes",
             FRAME_QUEUE_SIZE, USB_RX_BUF_NUM, MAX_PAYLOAD_SIZE);
    return &if_handle_g;
}

static int32_t esp_usb_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle)
{
    esp_err_t ret = ESP_OK;
    int32_t total_len = 0;
    uint16_t offset = 0;
    struct esp_payload_header *header = NULL;
    uint8_t *sendbuf = NULL;

    if (!handle || !buf_handle) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_FAIL;
    }

    if (!buf_handle->payload_len || !buf_handle->payload) {
        ESP_LOGE(TAG, "Invalid arguments, len:%" PRIu16, buf_handle->payload_len);
        return ESP_FAIL;
    }

    total_len = buf_handle->payload_len + sizeof(struct esp_payload_header);

    if (total_len > MAX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Payload size too large, max:%" PRIu16 ", current:%" PRId32, MAX_PAYLOAD_SIZE, total_len);
        return ESP_FAIL;
    }

    sendbuf = heap_caps_malloc(total_len, MALLOC_CAP_DMA);
    if (sendbuf == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail!");
        return ESP_FAIL;
    }

    header = (struct esp_payload_header *)(sendbuf);

    header->if_type = buf_handle->if_type;
    header->if_num = buf_handle->if_num;
    header->len = htole16(buf_handle->payload_len);
    header->reserved2 = buf_handle->flag;
    offset = sizeof(struct esp_payload_header);
    header->offset = htole16(offset);
    header->packet_type = buf_handle->pkt_type;
    header->checksum = 0;

    memcpy((uint8_t *)header + offset, buf_handle->payload, buf_handle->payload_len);

    header->checksum = htole16(compute_checksum(sendbuf, offset + buf_handle->payload_len));

    // Wait for TX completion - will block until TX is done
    if (xSemaphoreTake(usb_tx_sem, portMAX_DELAY) != pdTRUE) {
        free(sendbuf);
        return ESP_FAIL;
    }

    if (tud_vendor_n_write_available(0) > total_len) {
        ret = tud_vendor_n_write(0, sendbuf, total_len);
        if (ret == 0) {
            ESP_LOGE(TAG, "Failed to send USB packet, ret:%d", ret);
            free(sendbuf);
            return ESP_FAIL;
        }
        tud_vendor_n_flush(0);

        vTaskDelay(5 / portTICK_PERIOD_MS);
    } else {
        free(sendbuf);
        return ESP_FAIL;
    }

    free(sendbuf);
    return buf_handle->payload_len;
}

void tud_vendor_tx_cb(uint8_t itf, uint32_t sent_bytes)
{
    // Give semaphore to indicate TX completion
    xSemaphoreGive(usb_tx_sem);
}

static int esp_usb_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle)
{
    struct esp_payload_header *header = NULL;
    uint16_t len = 0;
    usb_frame_t received_frame;

    if (!if_handle) {
        ESP_LOGE(TAG, "Invalid arguments to esp_usb_read");
        return ESP_FAIL;
    }

    if (if_handle->state != ACTIVE) {
        ESP_LOGE(TAG, "USB not active");
        return ESP_FAIL;
    }

    // 从队列中读取数据，使用较短的超时时间
    if (xQueueReceive(frame_queue, &received_frame, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Timeout waiting for USB frame");
        return ESP_ERR_TIMEOUT;
    }

    // 直接使用队列中的数据指针
    buf_handle->payload = received_frame.data;
    buf_handle->payload_len = received_frame.len;
    buf_handle->priv_buffer_handle = received_frame.data;  // 设置私有缓冲区句柄
    buf_handle->free_buf_handle = release_rx_buffer;

    header = (struct esp_payload_header *) buf_handle->payload;
    len = le16toh(header->len) + le16toh(header->offset);

    buf_handle->if_type = header->if_type;
    buf_handle->if_num = header->if_num;

    return len;
}

static void esp_usb_deinit(interface_handle_t *handle)
{
    ESP_LOGI(TAG, "USB deinit");

    // Delete TX semaphore
    if (usb_tx_sem) {
        vSemaphoreDelete(usb_tx_sem);
        usb_tx_sem = NULL;
    }

    // 打印缓冲区使用统计
    ESP_LOGI(TAG, "Buffer stats - Allocated: %lu, Released: %lu, Emergency: %lu",
             buffer_alloc_count, buffer_release_count, emergency_alloc_count);

    // 清理接收队列
    if (frame_queue) {
        usb_frame_t frame;
        // 清空队列中的所有帧，释放预分配缓冲区
        while (xQueueReceive(frame_queue, &frame, 0) == pdTRUE) {
            if (frame.data) {
                release_rx_buffer(frame.data);
            }
        }
        vQueueDelete(frame_queue);
        frame_queue = NULL;
    }

    // 重置帧接收状态
    frame_active = false;
    received_total = 0;
    expected_total = 0;

    // 重置预分配缓冲区状态
    for (int i = 0; i < USB_RX_BUF_NUM; i++) {
        usb_rx_buffer_used[i] = false;
    }

    // 重置统计信息
    buffer_alloc_count = 0;
    buffer_release_count = 0;
    emergency_alloc_count = 0;
}