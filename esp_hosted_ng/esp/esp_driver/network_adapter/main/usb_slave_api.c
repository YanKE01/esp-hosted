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
#define FRAME_QUEUE_SIZE 15  // Increase queue size to accommodate more frames
#define USB_RX_BUF_NUM 10    // Increase buffer count to match queue size

#define TUSB_EVENT_EXIT         (1<<0)
#define TUSB_EVENT_EXIT_DONE    (1<<1)

static interface_context_t context;
static interface_handle_t if_handle_g;

// USB related structures
typedef struct {
    uint8_t *data;
    uint16_t len;
} usb_frame_t;

typedef struct {
    usb_phy_handle_t phy_hdl;
    SemaphoreHandle_t tx_sem;
    SemaphoreHandle_t buffer_mutex;
    QueueHandle_t frame_queue;
    TaskHandle_t tusb_device_task_handle;
    EventGroupHandle_t event_group;
    uint8_t usb_rx_buffer[USB_RX_BUF_NUM][MAX_PAYLOAD_SIZE];
    uint8_t usb_rx_temp_buffer[MAX_PAYLOAD_SIZE];
    bool usb_rx_buffer_used[USB_RX_BUF_NUM];
    uint32_t buffer_alloc_count;
    uint32_t buffer_release_count;
} usb_device_t;

static usb_device_t usb_device_g = {0};

static int32_t esp_usb_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle);
static int esp_usb_read(interface_handle_t *handle, interface_buffer_handle_t *buf_handle);
static void esp_usb_deinit(interface_handle_t *handle);
static interface_handle_t *esp_usb_init(void);

static const char TAG[] = "FW_USB";

// Get available pre-allocated buffer
static uint8_t* get_available_rx_buffer(void)
{
    uint8_t *buffer = NULL;

    if (usb_device_g.buffer_mutex == NULL) {
        ESP_LOGE(TAG, "Buffer mutex not initialized");
        return NULL;
    }

    /* Take mutex to protect buffer state */
    if (xSemaphoreTake(usb_device_g.buffer_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take buffer mutex");
        return NULL;
    }

    for (int i = 0; i < USB_RX_BUF_NUM; i++) {
        if (!usb_device_g.usb_rx_buffer_used[i]) {
            usb_device_g.usb_rx_buffer_used[i] = true;
            usb_device_g.buffer_alloc_count++;
            ESP_LOGD(TAG, "Allocated RX buffer %d (total: %lu)", i, usb_device_g.buffer_alloc_count);
            buffer = usb_device_g.usb_rx_buffer[i];
            xSemaphoreGive(usb_device_g.buffer_mutex);
            return buffer;
        }
    }

    /* If all buffers are in use, force release of the first buffer */
    ESP_LOGW(TAG, "All buffers in use, forcing release of buffer 0");
    memset(usb_device_g.usb_rx_buffer[0], 0, MAX_PAYLOAD_SIZE);
    usb_device_g.usb_rx_buffer_used[0] = true;
    usb_device_g.buffer_alloc_count++;
    buffer = usb_device_g.usb_rx_buffer[0];
    xSemaphoreGive(usb_device_g.buffer_mutex);
    return buffer;
}

// Release pre-allocated buffer
static void release_rx_buffer(void *buffer)
{
    if (usb_device_g.buffer_mutex == NULL) {
        ESP_LOGE(TAG, "Buffer mutex not initialized");
        return;
    }

    /* Take mutex to protect buffer state */
    if (xSemaphoreTake(usb_device_g.buffer_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take buffer mutex");
        return;
    }

    for (int i = 0; i < USB_RX_BUF_NUM; i++) {
        if (buffer == usb_device_g.usb_rx_buffer[i]) {
            usb_device_g.usb_rx_buffer_used[i] = false;
            usb_device_g.buffer_release_count++;
            ESP_LOGD(TAG, "Released RX buffer %d (total: %lu)", i, usb_device_g.buffer_release_count);
            xSemaphoreGive(usb_device_g.buffer_mutex);
            return;
        }
    }

    ESP_LOGE(TAG, "Attempted to release unknown buffer");
    xSemaphoreGive(usb_device_g.buffer_mutex);
}

void tud_mount_cb(void)
{
    ESP_LOGI(TAG, "USB device mounted");
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
    ESP_LOGI(TAG, "Using USB interface");
    memset(&context, 0, sizeof(context));

    context.type = USB;
    context.if_ops = &if_ops;
    context.event_handler = event_handler;

    return &context;
}

int interface_remove_driver()
{
    memset(&context, 0, sizeof(context));
    return 0;
}

esp_err_t send_bootup_event_to_host(uint32_t cap)
{
    struct esp_payload_header *header = NULL;
    struct esp_internal_bootup_event *event = NULL;
    struct fw_data * fw_p = NULL;
    interface_buffer_handle_t buf_handle = {0};
    uint8_t * pos = NULL;
    esp_err_t ret = ESP_OK;
    uint16_t len = 0;

    ESP_LOGI(TAG, "Sending bootup event to host, capabilities: 0x%lx", cap);
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
    *pos = LENGTH_4_BYTE;                 pos++; len++;
    *(uint32_t *)pos = htole32(cap);      pos += 4; len += 4;

    /* TLV - FW data */
    *pos = ESP_BOOTUP_FW_DATA;            pos++; len++;
    *pos = sizeof(struct fw_data);        pos++; len++;
    fw_p = (struct fw_data *) pos;
    /* core0 sufficient now */
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

    /* Construct USB packet */
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

    /* Wait for TX completion - will block until TX is done (tud_vendor_tx_cb gives the semaphore) */
    xSemaphoreTake(usb_device_g.tx_sem, portMAX_DELAY);

    free(buf_handle.payload);

    return ESP_OK;
}

// USB Driver Related Functions
static void usb_phy_init(void)
{
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .target = USB_PHY_TARGET_INT
    };
    usb_new_phy(&phy_conf, &usb_device_g.phy_hdl);
}

static void tusb_device_task(void *pvParameters)
{
    while (1) {
        EventBits_t uxBits = xEventGroupGetBits(usb_device_g.event_group);
        if (uxBits & TUSB_EVENT_EXIT) {
            ESP_LOGI(TAG, "TUSB task exit");
            break;
        }
        tud_task();
    }
    xEventGroupSetBits(usb_device_g.event_group, TUSB_EVENT_EXIT_DONE);
    vTaskDelete(NULL);
}

static int process_usb_rx(uint8_t const *buffer, uint16_t bufsize)
{
    int ret = 0;
    struct esp_payload_header *header = NULL;
    uint16_t len = 0, offset = 0;
    usb_frame_t frame;
    uint8_t *rx_buffer = NULL;

    /* Validate received buffer */
    if (!buffer || bufsize < sizeof(struct esp_payload_header)) {
        ESP_LOGE(TAG, "%s: Invalid params, bufsize=%u", __func__, bufsize);
        return -1;
    }

    header = (struct esp_payload_header *) buffer;
    len = le16toh(header->len);
    offset = le16toh(header->offset);

    if (!len || (len > MAX_PAYLOAD_SIZE)) {
        ESP_LOGW(TAG, "Invalid length: %u", len);
        return -1;
    }

    if (len + offset > MAX_PAYLOAD_SIZE) {
        ESP_LOGW(TAG, "Total size exceeds max: %u + %u > %u", offset, len, MAX_PAYLOAD_SIZE);
        return -1;
    }

    /* Buffer is valid - allocate RX buffer */
    rx_buffer = get_available_rx_buffer();
    if (!rx_buffer) {
        ESP_LOGE(TAG, "No available RX buffer");
        return -1;
    }

    /* Copy data to RX buffer */
    memcpy(rx_buffer, buffer, len + offset);

    /* Prepare frame for queue */
    frame.data = rx_buffer;
    frame.len = len + offset;

    /* Send to queue - use timeout to avoid blocking forever */
    ret = xQueueSend(usb_device_g.frame_queue, &frame, portMAX_DELAY);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Failed to queue frame");
        release_rx_buffer(rx_buffer);
        return -1;
    }

    return 0;
}

void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize)
{
    (void) buffer;
    (void) bufsize;

    while (tud_vendor_n_available(itf)) {
        /* Use global buffer instead of heap allocation for better performance */
        uint32_t read_len = tud_vendor_n_read(itf, usb_device_g.usb_rx_temp_buffer, MAX_PAYLOAD_SIZE);

        if (read_len == 0) {
            break;
        }

        if (process_usb_rx(usb_device_g.usb_rx_temp_buffer, (uint16_t)read_len) != 0) {
            ESP_LOGW(TAG, "Failed to process USB RX data, len=%lu", read_len);
            ESP_LOG_BUFFER_HEXDUMP(TAG, usb_device_g.usb_rx_temp_buffer, read_len, ESP_LOG_INFO);
        }
    }
}

static interface_handle_t *esp_usb_init(void)
{
    /* Create TX semaphore */
    usb_device_g.tx_sem = xSemaphoreCreateBinary();
    if (!usb_device_g.tx_sem) {
        ESP_LOGE(TAG, "Failed to create TX semaphore");
        return NULL;
    }

    /* Create buffer mutex */
    usb_device_g.buffer_mutex = xSemaphoreCreateMutex();
    if (!usb_device_g.buffer_mutex) {
        ESP_LOGE(TAG, "Failed to create buffer mutex");
        vSemaphoreDelete(usb_device_g.tx_sem);
        return NULL;
    }

    /* Create event group */
    usb_device_g.event_group = xEventGroupCreate();
    if (!usb_device_g.event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        vSemaphoreDelete(usb_device_g.buffer_mutex);
        vSemaphoreDelete(usb_device_g.tx_sem);
        return NULL;
    }

    /* Initialize USB phy */
    usb_phy_init();
    if (!tusb_init()) {
        /* Clean up in reverse order of creation */
        if (usb_device_g.phy_hdl) {
            usb_del_phy(usb_device_g.phy_hdl);
            usb_device_g.phy_hdl = NULL;
        }
        vEventGroupDelete(usb_device_g.event_group);
        usb_device_g.event_group = NULL;
        vSemaphoreDelete(usb_device_g.buffer_mutex);
        usb_device_g.buffer_mutex = NULL;
        vSemaphoreDelete(usb_device_g.tx_sem);
        usb_device_g.tx_sem = NULL;
        ESP_LOGE(TAG, "TinyUSB init failed");
        return NULL;
    }
    xSemaphoreGive(usb_device_g.tx_sem);

    xTaskCreate(tusb_device_task, "tusb_device_task", 10 * 1024, NULL, 14, &usb_device_g.tusb_device_task_handle);

    /* Create receive queue */
    usb_device_g.frame_queue = xQueueCreate(FRAME_QUEUE_SIZE, sizeof(usb_frame_t));
    if (!usb_device_g.frame_queue) {
        ESP_LOGE(TAG, "Failed to create frame queue");
        return NULL;
    }

    /* Initialize pre-allocated buffer states */
    for (int i = 0; i < USB_RX_BUF_NUM; i++) {
        usb_device_g.usb_rx_buffer_used[i] = false;
    }

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
    header->flags  = buf_handle->flag;
    offset = sizeof(struct esp_payload_header);
    header->offset = htole16(offset);
    header->packet_type = buf_handle->pkt_type;
    header->checksum = 0;

    memcpy((uint8_t *)header + offset, buf_handle->payload, buf_handle->payload_len);

    header->checksum = htole16(compute_checksum(sendbuf, offset + buf_handle->payload_len));

    /* Acquire semaphore to mark TX in-progress */
    if (xSemaphoreTake(usb_device_g.tx_sem, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take TX semaphore");
        free(sendbuf);
        return ESP_FAIL;
    }

    /* Allow write when available bytes are >= required length */
    if (tud_vendor_n_write_available(0) >= total_len) {
        ret = tud_vendor_n_write(0, sendbuf, total_len);
        if (ret == 0) {
            ESP_LOGE(TAG, "Failed to send USB packet, ret:%d", ret);
            /* Release semaphore to avoid deadlock */
            xSemaphoreGive(usb_device_g.tx_sem);
            free(sendbuf);
            return ESP_FAIL;
        }
        tud_vendor_n_flush(0);
    } else {
        ESP_LOGW(TAG, "USB TX buffer not available: need=%" PRId32 " available=%" PRId32 "", total_len, tud_vendor_n_write_available(0));
        /* Release semaphore and fail */
        xSemaphoreGive(usb_device_g.tx_sem);
        free(sendbuf);
        return ESP_FAIL;
    }

    free(sendbuf);
    return buf_handle->payload_len;
}

void tud_vendor_tx_cb(uint8_t itf, uint32_t sent_bytes)
{
    /* Give semaphore to indicate TX completion */
    xSemaphoreGive(usb_device_g.tx_sem);
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

    /* Read data from queue */
    if (xQueueReceive(usb_device_g.frame_queue, &received_frame, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Timeout waiting for USB frame");
        return ESP_ERR_TIMEOUT;
    }

    /* Use data pointer from queue directly */
    buf_handle->payload = received_frame.data;
    buf_handle->payload_len = received_frame.len;
    buf_handle->priv_buffer_handle = received_frame.data;
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

    /* Print buffer usage statistics */
    ESP_LOGI(TAG, "Buffer stats - Allocated: %lu, Released: %lu",
             usb_device_g.buffer_alloc_count, usb_device_g.buffer_release_count);

    /* Clean up receive queue */
    if (usb_device_g.frame_queue) {
        usb_frame_t frame;
        /* Clear all frames in queue and release pre-allocated buffers */
        while (xQueueReceive(usb_device_g.frame_queue, &frame, 0) == pdTRUE) {
            if (frame.data) {
                release_rx_buffer(frame.data);
            }
        }
        vQueueDelete(usb_device_g.frame_queue);
        usb_device_g.frame_queue = NULL;
    }

    /* Reset pre-allocated buffer states (with mutex protection) */
    if (usb_device_g.buffer_mutex) {
        if (xSemaphoreTake(usb_device_g.buffer_mutex, portMAX_DELAY) == pdTRUE) {
            for (int i = 0; i < USB_RX_BUF_NUM; i++) {
                usb_device_g.usb_rx_buffer_used[i] = false;
                memset(usb_device_g.usb_rx_buffer[i], 0, MAX_PAYLOAD_SIZE);
            }
            // Reset statistics
            usb_device_g.buffer_alloc_count = 0;
            usb_device_g.buffer_release_count = 0;
            xSemaphoreGive(usb_device_g.buffer_mutex);
        }
    }

    /* Delete buffer mutex */
    if (usb_device_g.buffer_mutex) {
        vSemaphoreDelete(usb_device_g.buffer_mutex);
        usb_device_g.buffer_mutex = NULL;
    }

    /* Delete TX semaphore */
    if (usb_device_g.tx_sem) {
        vSemaphoreDelete(usb_device_g.tx_sem);
        usb_device_g.tx_sem = NULL;
    }

    /* Stop TinyUSB task */
    if (usb_device_g.tusb_device_task_handle) {
        /* Set exit flag to notify task to exit */
        xEventGroupSetBits(usb_device_g.event_group, TUSB_EVENT_EXIT);
        /* Wait for task to exit gracefully */
        EventBits_t bits = xEventGroupWaitBits(usb_device_g.event_group, TUSB_EVENT_EXIT_DONE, pdTRUE, pdTRUE, pdMS_TO_TICKS(5000));
        if (!(bits & TUSB_EVENT_EXIT_DONE)) {
            ESP_LOGW(TAG, "TinyUSB task exit timeout (5s), force delete");
            if (usb_device_g.tusb_device_task_handle) {
                vTaskDelete(usb_device_g.tusb_device_task_handle);
                usb_device_g.tusb_device_task_handle = NULL;
            }
        } else {
            usb_device_g.tusb_device_task_handle = NULL;
        }
    }

    /* Delete event group */
    if (usb_device_g.event_group) {
        vEventGroupDelete(usb_device_g.event_group);
        usb_device_g.event_group = NULL;
    }

    /* Teardown USB stack */
    tusb_teardown();
    if (usb_device_g.phy_hdl) {
        usb_del_phy(usb_device_g.phy_hdl);
        usb_device_g.phy_hdl = NULL;
    }

    ESP_LOGI(TAG, "USB deinit completed");
}