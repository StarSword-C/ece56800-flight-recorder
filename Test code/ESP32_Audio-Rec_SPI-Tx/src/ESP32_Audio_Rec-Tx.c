// main.c
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_check.h"

#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"

// ===================== User Configuration =====================

#define TAG "AUDIO_SPI_PROTO"

// Audio
#define SAMPLE_RATE_HZ              16000
#define AUDIO_BLOCK_SAMPLES         256                     // 256 samples/block
#define AUDIO_BLOCK_BYTES           (AUDIO_BLOCK_SAMPLES * sizeof(int16_t))
#define ADC_FRAME_BYTES             256                     // continuous ADC frame size
#define ADC_MAX_STORE_BYTES         2048

// SPI pins (Adafruit ESP32 Feather V2)
#define PIN_NUM_MISO                GPIO_NUM_21            // ESP32 -> FPGA
#define PIN_NUM_MOSI                GPIO_NUM_19            // FPGA  -> ESP32
#define PIN_NUM_SCLK                GPIO_NUM_5
#define PIN_NUM_CS                  GPIO_NUM_4

// Other board pins
#define LED_GPIO                    GPIO_NUM_13
#define BUTTON_GPIO                 GPIO_NUM_38            // active low, onboard pull-up on board

// Timing / power
#define MASTER_CONNECT_TIMEOUT_MS   5000                   // after wake, how long to wait for first SPI transaction
#define MASTER_IDLE_TIMEOUT_MS      2000                   // after connected, sleep if no more transactions
#define BUTTON_DEBOUNCE_MS          200

// SPI
#define SPI_USED_HOST               SPI2_HOST              // ESP32: SPI2_HOST or SPI3_HOST
#define SPI_MODE                    0

// Queue / buffering
#define NUM_AUDIO_BLOCKS            4

// ===================== Globals =====================

static adc_continuous_handle_t s_adc_handle = NULL;

static TaskHandle_t s_adc_task_handle = NULL;
static QueueHandle_t s_ready_block_q = NULL;               // filled audio blocks ready for SPI
static QueueHandle_t s_free_block_q  = NULL;               // free audio blocks available
static QueueHandle_t s_button_q      = NULL;               // button ISR events

static volatile bool s_streaming_enabled = false;
static volatile bool s_master_connected = false;
static volatile bool s_adc_started = false;

static int64_t s_session_start_us = 0;
static int64_t s_last_spi_activity_us = 0;
static int64_t s_last_button_irq_us = 0;

// Simple DC estimate in raw ADC counts
static int32_t s_dc_estimate = 2048;

// Audio buffers
static int16_t s_audio_blocks[NUM_AUDIO_BLOCKS][AUDIO_BLOCK_SAMPLES];
static uint8_t s_spi_rx_dummy[AUDIO_BLOCK_BYTES];
static int16_t s_silence_block[AUDIO_BLOCK_SAMPLES];

// ADC raw frame buffer
static uint8_t s_adc_frame[ADC_FRAME_BYTES];

// ===================== Helper Functions =====================

static void led_set(bool on)
{
    gpio_set_level(LED_GPIO, on ? 1 : 0);
}

/*static void drain_queue_to_free(QueueHandle_t src_q, QueueHandle_t free_q) // unused
{
    int idx;
    while (xQueueReceive(src_q, &idx, 0) == pdTRUE) {
        xQueueSend(free_q, &idx, 0);
    }
}*/

static void reset_audio_queues(void)
{
    xQueueReset(s_ready_block_q);
    xQueueReset(s_free_block_q);

    for (int i = 0; i < NUM_AUDIO_BLOCKS; i++) {
        xQueueSend(s_free_block_q, &i, 0);
    }
}

static esp_err_t adc_start_if_needed(void)
{
    if (!s_adc_started) {
        ESP_RETURN_ON_ERROR(adc_continuous_start(s_adc_handle), TAG, "adc_continuous_start failed");
        s_adc_started = true;
        ESP_LOGI(TAG, "ADC continuous sampling started");
    }
    return ESP_OK;
}

static void adc_stop_if_needed(void)
{
    if (s_adc_started) {
        esp_err_t err = adc_continuous_stop(s_adc_handle);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "adc_continuous_stop returned %s", esp_err_to_name(err));
        }
        s_adc_started = false;
        ESP_LOGI(TAG, "ADC continuous sampling stopped");
    }
}

static void start_streaming_session(void)
{
    s_streaming_enabled = false;           // hold off while we reset state
    s_master_connected = false;
    adc_stop_if_needed();

    s_dc_estimate = 2048;
    memset(s_silence_block, 0, sizeof(s_silence_block));
    memset(s_spi_rx_dummy, 0, sizeof(s_spi_rx_dummy));

    reset_audio_queues();

    s_session_start_us = esp_timer_get_time();
    s_last_spi_activity_us = s_session_start_us;

    s_streaming_enabled = true;
    led_set(true);

    ESP_LOGI(TAG, "Streaming session armed; waiting for FPGA master...");
}

static void stop_streaming_session(void)
{
    s_streaming_enabled = false;
    s_master_connected = false;
    adc_stop_if_needed();
    led_set(false);
    ESP_LOGI(TAG, "Streaming session stopped");
}

// ===================== ADC ISR Callback =====================

static bool adc_conv_done_cb(adc_continuous_handle_t handle,
                             const adc_continuous_evt_data_t *edata,
                             void *user_data)
{
    BaseType_t high_task_woken = pdFALSE;
    if (s_adc_task_handle != NULL) {
        vTaskNotifyGiveFromISR(s_adc_task_handle, &high_task_woken);
    }
    return (high_task_woken == pdTRUE);
}

// ===================== GPIO ISR =====================

static void IRAM_ATTR button_isr_handler(void *arg)
{
    int64_t now_us = esp_timer_get_time();

    // crude debounce in ISR
    if ((now_us - s_last_button_irq_us) < (BUTTON_DEBOUNCE_MS * 1000LL)) {
        return;
    }
    s_last_button_irq_us = now_us;

    uint32_t evt = 1;
    BaseType_t high_task_woken = pdFALSE;
    xQueueSendFromISR(s_button_q, &evt, &high_task_woken);
    if (high_task_woken) {
        portYIELD_FROM_ISR();
    }
}

// ===================== Init Functions =====================

static esp_err_t init_gpio(void)
{
    // LED
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&led_cfg), TAG, "LED gpio_config failed");
    led_set(false);

    // Button: GPIO38 is input-only. Board provides pull-up.
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&btn_cfg), TAG, "Button gpio_config failed");

    ESP_RETURN_ON_ERROR(gpio_install_isr_service(0), TAG, "gpio_install_isr_service failed");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL), TAG, "button ISR add failed");

    return ESP_OK;
}

static esp_err_t init_spi_slave(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = AUDIO_BLOCK_BYTES
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = SPI_MODE,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 2,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL
    };

    // Use DMA auto channel selection
    ESP_RETURN_ON_ERROR(spi_slave_initialize(SPI_USED_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO),
                        TAG, "spi_slave_initialize failed");

    return ESP_OK;
}

static esp_err_t init_adc_continuous(void)
{
    adc_continuous_handle_cfg_t adc_handle_cfg = {
        .max_store_buf_size = ADC_MAX_STORE_BYTES,
        .conv_frame_size = ADC_FRAME_BYTES
    };
    ESP_RETURN_ON_ERROR(adc_continuous_new_handle(&adc_handle_cfg, &s_adc_handle),
                        TAG, "adc_continuous_new_handle failed");

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_conv_done_cb,
    };
    ESP_RETURN_ON_ERROR(adc_continuous_register_event_callbacks(s_adc_handle, &cbs, NULL),
                        TAG, "adc callback registration failed");

    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN_DB_12,
        .channel = ADC_CHANNEL_0,          // ADC1_CH0 -> GPIO36
        .unit = ADC_UNIT_1,
        .bit_width = ADC_BITWIDTH_12
    };

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_RATE_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        .pattern_num = 1,
        .adc_pattern = &pattern
    };

    ESP_RETURN_ON_ERROR(adc_continuous_config(s_adc_handle, &dig_cfg),
                        TAG, "adc_continuous_config failed");

    return ESP_OK;
}

// ===================== Tasks =====================

static void adc_task(void *arg)
{
    int fill_idx = -1;
    size_t fill_pos = 0;

    while (1) {
        // Wait until ISR says a frame is ready
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!s_streaming_enabled || !s_master_connected || !s_adc_started) {
            continue;
        }

        while (1) {
            uint32_t out_len = 0;
            esp_err_t err = adc_continuous_read(s_adc_handle,
                                                s_adc_frame,
                                                sizeof(s_adc_frame),
                                                &out_len,
                                                0);

            if (err == ESP_ERR_TIMEOUT || out_len == 0) {
                break;
            }

            if (err != ESP_OK) {
                ESP_LOGW(TAG, "adc_continuous_read: %s", esp_err_to_name(err));
                break;
            }

            // Parse ADC results
            for (uint32_t i = 0; i < out_len; i += sizeof(adc_digi_output_data_t)) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t *)&s_adc_frame[i];

                uint32_t raw = p->type1.data & 0x0FFF;
                uint32_t chan = p->type1.channel;

                // Safety check: only keep ADC1_CH0 samples
                if (chan != 0 || chan != ADC_CHANNEL_0) {
                    continue;
                }

                // Get a free block if we don't currently have one
                if (fill_idx < 0) {
                    if (xQueueReceive(s_free_block_q, &fill_idx, 0) != pdTRUE) {
                        // No free buffer available; drop sample
                        continue;
                    }
                    fill_pos = 0;
                }

                // Simple running DC estimate (slow IIR)
                // Good enough for prototyping voice capture.
                s_dc_estimate = ((s_dc_estimate * 1023) + (int32_t)raw) / 1024;

                // Convert unsigned 12-bit raw to signed 16-bit PCM
                int32_t centered = (int32_t)raw - s_dc_estimate; // about +/-2048
                int16_t pcm = (int16_t)(centered << 4);          // scale 12-bit-ish to 16-bit container

                s_audio_blocks[fill_idx][fill_pos++] = pcm;

                if (fill_pos >= AUDIO_BLOCK_SAMPLES) {
                    // Finished a block; queue it for SPI transmission
                    if (xQueueSend(s_ready_block_q, &fill_idx, 0) != pdTRUE) {
                        // If ready queue is full, recycle the block
                        xQueueSend(s_free_block_q, &fill_idx, 0);
                    }
                    fill_idx = -1;
                    fill_pos = 0;
                }
            }
        }
    }
}

static void spi_task(void *arg)
{
    while (1) {
        if (!s_streaming_enabled) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        int block_idx = -1;
        const int16_t *tx_ptr = s_silence_block;

        // Before master connects, send silence blocks.
        // After it connects, send audio blocks when available.
        if (s_master_connected) {
            if (xQueueReceive(s_ready_block_q, &block_idx, pdMS_TO_TICKS(20)) == pdTRUE) {
                tx_ptr = s_audio_blocks[block_idx];
            } else {
                tx_ptr = s_silence_block;
            }
        } else {
            tx_ptr = s_silence_block;
        }

        spi_slave_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = AUDIO_BLOCK_BYTES * 8;
        t.tx_buffer = tx_ptr;
        t.rx_buffer = s_spi_rx_dummy;

        esp_err_t err = spi_slave_transmit(SPI_USED_HOST, &t, pdMS_TO_TICKS(100));

        if (err == ESP_OK) {
            s_last_spi_activity_us = esp_timer_get_time();

            if (!s_master_connected) {
                s_master_connected = true;
                ESP_LOGI(TAG, "FPGA master detected on SPI");
                if (adc_start_if_needed() != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to start ADC after master connect");
                    stop_streaming_session();
                }
            }

            if (block_idx >= 0) {
                xQueueSend(s_free_block_q, &block_idx, 0);
            }
        } else if (err == ESP_ERR_TIMEOUT) {
            // No SPI activity in this window; recycle any borrowed block
            if (block_idx >= 0) {
                xQueueSend(s_free_block_q, &block_idx, 0);
            }
        } else {
            ESP_LOGW(TAG, "spi_slave_transmit: %s", esp_err_to_name(err));
            if (block_idx >= 0) {
                xQueueSend(s_free_block_q, &block_idx, 0);
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

static void button_task(void *arg)
{
    uint32_t evt;

    while (1) {
        if (xQueueReceive(s_button_q, &evt, portMAX_DELAY) == pdTRUE) {
            if (s_streaming_enabled) {
                ESP_LOGI(TAG, "Button pressed: stopping session");
                stop_streaming_session();
            }
        }
    }
}

static void power_task(void *arg)
{
    while (1) {
        if (!s_streaming_enabled) {
            // Sleep until button press wakes the chip
            ESP_LOGI(TAG, "Entering light sleep. Press button to wake/start.");
            led_set(false);

            // Wake on button low
            esp_sleep_enable_ext0_wakeup(BUTTON_GPIO, 0);

            // Small delay so logs flush cleanly
            vTaskDelay(pdMS_TO_TICKS(50));
            esp_light_sleep_start();

            esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
            if (cause == ESP_SLEEP_WAKEUP_EXT0) {
                ESP_LOGI(TAG, "Woke from button press");
                // Wait for button release
                while (gpio_get_level(BUTTON_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
                start_streaming_session();
            } else {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        } else {
            int64_t now_us = esp_timer_get_time();

            if (!s_master_connected) {
                if ((now_us - s_session_start_us) > (MASTER_CONNECT_TIMEOUT_MS * 1000LL)) {
                    ESP_LOGW(TAG, "Master did not connect in time; sleeping");
                    stop_streaming_session();
                }
            } else {
                if ((now_us - s_last_spi_activity_us) > (MASTER_IDLE_TIMEOUT_MS * 1000LL)) {
                    ESP_LOGW(TAG, "SPI idle timeout; sleeping");
                    stop_streaming_session();
                }
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// ===================== app_main =====================

void app_main(void)
{
    ESP_ERROR_CHECK(init_gpio());
    ESP_ERROR_CHECK(init_spi_slave());
    ESP_ERROR_CHECK(init_adc_continuous());

    s_ready_block_q = xQueueCreate(NUM_AUDIO_BLOCKS, sizeof(int));
    s_free_block_q  = xQueueCreate(NUM_AUDIO_BLOCKS, sizeof(int));
    s_button_q      = xQueueCreate(4, sizeof(uint32_t));

    if (!s_ready_block_q || !s_free_block_q || !s_button_q) {
        ESP_LOGE(TAG, "Queue creation failed");
        while (1) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    reset_audio_queues();

    xTaskCreate(adc_task,    "adc_task",    4096, NULL, 10, &s_adc_task_handle);
    xTaskCreate(spi_task,    "spi_task",    4096, NULL,  9, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL,  8, NULL);
    xTaskCreate(power_task,  "power_task",  3072, NULL,  7, NULL);

    ESP_LOGI(TAG, "Prototype ready");
}