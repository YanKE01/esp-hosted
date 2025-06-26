#include "sdkconfig.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
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
#define FRAME_QUEUE_SIZE 5

static interface_context_t context;
static interface_handle_t if_handle_g;

// 用于解析USB包
static uint8_t frame_buffer[MAX_PAYLOAD_SIZE] = {0};
static uint16_t received_total = 0;
static uint16_t expected_total = 0;
static uint16_t expected_crc16 = 0;
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

typedef struct __attribute__((packed))
{
    uint16_t total_len;
    uint16_t crc16;
} usb_packet_header_t;

static const char TAG[] = "FW_USB";

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

static void IRAM_ATTR esp_usb_free(void *handle)
{
    if (handle) {
        free(handle);
        handle = NULL;
    }
}

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len, uint16_t init)
{
    uint16_t crc = init;
    while (len--) {
        crc ^= (*data++) << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
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

#if CONFIG_ESP_SDIO_CHECKSUM
    header->checksum = htole16(compute_checksum(buf_handle.payload, buf_handle.payload_len));
#endif

    // 构造USB包
    int32_t total_len = buf_handle.payload_len + sizeof(usb_packet_header_t);

    if (total_len > MAX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Payload size too large, max:%d, current:%ld", MAX_PAYLOAD_SIZE, total_len);
        free(buf_handle.payload);
        return ESP_FAIL;
    }

    uint8_t *sendbuf = heap_caps_malloc(total_len, MALLOC_CAP_DMA);

    if (!sendbuf) {
        ESP_LOGE(TAG, "Failed to allocate USB-wrapped buffer");
        free(buf_handle.payload);
        return ESP_FAIL;
    }

    usb_packet_header_t *usb_header = (usb_packet_header_t *)sendbuf;
    usb_header->total_len = htole16(buf_handle.payload_len);

    usb_header->crc16 = htole16(crc16_ccitt(buf_handle.payload, buf_handle.payload_len, 0x0000));

    memcpy(sendbuf + sizeof(usb_packet_header_t), buf_handle.payload, buf_handle.payload_len);

    ESP_LOGI(TAG, "Send_bootup_event_to_host. Len: %ld, crc: 0x%04x", total_len, usb_header->crc16);

    ret = tud_vendor_n_write(0, sendbuf, total_len);

    if (ret < 0) {
        ESP_LOGE(TAG, "Failed to send USB packet, ret:%d", ret);
        free(sendbuf);
        free(buf_handle.payload);
        return ESP_FAIL;
    }

    tud_vendor_n_flush(0); // 立马发出

    free(sendbuf);
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
        uint8_t temp_buf[VENDOR_BUF_SIZE];
        int read_len = tud_vendor_n_read(itf, temp_buf, sizeof(temp_buf));
        if (read_len <= 0) {
            return;
        }

        if (!frame_active && read_len >= HEADER_SIZE) {
            // Decode header
            usb_packet_header_t *hdr = (usb_packet_header_t *)temp_buf;
            expected_total = hdr->total_len;
            expected_crc16 = hdr->crc16;

            ESP_LOGI(TAG, "Header: total_len=%d, crc16=0x%04X", expected_total, expected_crc16);

            if (expected_total > MAX_PAYLOAD_SIZE) {
                ESP_LOGE(TAG, "Payload too large");
                return;
            }

            frame_active = true;
            received_total = 0;

            // Copy data from first packet (excluding header)
            int data_len = read_len - HEADER_SIZE;
            if (data_len > 0) {
                memcpy(frame_buffer, temp_buf + HEADER_SIZE, data_len);
                received_total = data_len;
            }
        } else if (frame_active) {
            // Concatenate remaining packet data
            int remain = expected_total - received_total;
            int to_copy = (read_len > remain) ? remain : read_len;

            if (to_copy > 0) {
                memcpy(frame_buffer + received_total, temp_buf, to_copy);
                received_total += to_copy;
            }
        }

        // 验证数据的有效性
        if (frame_active && received_total >= expected_total) {
            ESP_LOGI(TAG, "Frame received (%d bytes), computing CRC...", expected_total);

            uint16_t calc_crc = crc16_ccitt(frame_buffer, expected_total, 0x0000);
            if (calc_crc == expected_crc16) {
                ESP_LOGI(TAG, "CRC OK (0x%04X)", calc_crc);
            } else {
                ESP_LOGE(TAG, "CRC MISMATCH! expected 0x%04X, got 0x%04X", expected_crc16, calc_crc);
                return;
            }

            usb_frame_t frame;
            frame.data = malloc(expected_total);
            if (!frame.data) {
                ESP_LOGE(TAG, "Failed to alloc USB frame");
                return;
            }

            memcpy(frame.data, frame_buffer, expected_total);
            frame.len = expected_total;
            xQueueSend(frame_queue, &frame, 0);

            // Reset statet
            frame_active = false;
            received_total = 0;
            expected_total = 0;
            expected_crc16 = 0;
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
    xTaskCreate(tusb_device_task, "tusb_device_task", 10 * 1024, NULL, 5, NULL);

    frame_queue = xQueueCreate(FRAME_QUEUE_SIZE, sizeof(usb_frame_t));
    if (!frame_queue) {
        ESP_LOGE(TAG, "Failed to create frame queue");
        return NULL;
    }

    memset(&if_handle_g, 0, sizeof(if_handle_g));
    if_handle_g.state = INIT;
    return &if_handle_g;
}

static int32_t esp_usb_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle)
{
    esp_err_t ret = ESP_OK;
    int32_t total_len = 0, sdio_len = 0;
    uint16_t offset = 0;
    struct esp_payload_header *header = NULL;
    uint8_t *sendbuf = NULL;

    if (!handle || !buf_handle) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_FAIL;
    }

    if (!buf_handle->payload_len || !buf_handle->payload) {
        ESP_LOGE(TAG, "Invalid arguments, len:%d", buf_handle->payload_len);
        return ESP_FAIL;
    }

    sdio_len = buf_handle->payload_len + sizeof(struct esp_payload_header);
    total_len = sdio_len + sizeof(usb_packet_header_t);

    if (total_len > MAX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Payload size too large, max:%d, current:%ld", MAX_PAYLOAD_SIZE, total_len);
        return ESP_FAIL;
    }

    sendbuf = heap_caps_malloc(total_len, MALLOC_CAP_DMA);
    if (sendbuf == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail!");
        return ESP_FAIL;
    }

    memset(sendbuf, 0, total_len);

    usb_packet_header_t *usb_header  = (usb_packet_header_t *)sendbuf;
    usb_header->total_len = htole16(sdio_len);  // 不含 USB header 本身
    usb_header->crc16 = 0;                     // 最后计算

    header = (struct esp_payload_header *)(sendbuf + sizeof(usb_packet_header_t));

    header->if_type = buf_handle->if_type;
    header->if_num = buf_handle->if_num;
    header->len = htole16(buf_handle->payload_len);
    header->reserved2 = buf_handle->flag;
    offset = sizeof(struct esp_payload_header);
    header->offset = htole16(offset);
    header->packet_type = buf_handle->pkt_type;

    memcpy((uint8_t *)header + offset, buf_handle->payload, buf_handle->payload_len);

    uint8_t *crc_data = sendbuf + sizeof(usb_packet_header_t);
    usb_header->crc16 = htole16(crc16_ccitt(crc_data, sdio_len, 0x0000));

    ret = tud_vendor_n_write(0, sendbuf, total_len);
    if (ret < 0) {
        ESP_LOGE(TAG, "Failed to send USB packet, ret:%d", ret);
        free(sendbuf);
        return ESP_FAIL;
    }
    tud_vendor_n_flush(0);

    free(sendbuf);
    return 0;
}

static int esp_usb_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle)
{
    struct esp_payload_header *header = NULL;
    uint16_t len = 0;

    if (!if_handle) {
        ESP_LOGE(TAG, "Invalid arguments to sdio_read");
        return ESP_FAIL;
    }

    if (if_handle->state != ACTIVE) {
        ESP_LOGE(TAG, "USB not active");
        return ESP_FAIL;
    }
    usb_frame_t received_frame;

    if (xQueueReceive(frame_queue, &received_frame, 200) != pdTRUE) {
        ESP_LOGW(TAG, "Timeout waiting for USB frame");
        return ESP_ERR_TIMEOUT;
    }

    buf_handle->payload = received_frame.data;
    buf_handle->payload_len = received_frame.len;
    buf_handle->free_buf_handle = esp_usb_free;

    header = (struct esp_payload_header *) buf_handle->payload;
    len = le16toh(header->len) + le16toh(header->offset);

    buf_handle->if_type = header->if_type;
    buf_handle->if_num = header->if_num;

    return len;
}

static void esp_usb_deinit(interface_handle_t *handle)
{
    ESP_LOGI(TAG, "USB deinit");
}