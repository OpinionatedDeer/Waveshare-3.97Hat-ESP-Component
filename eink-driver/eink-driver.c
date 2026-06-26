/**
 * @file eink-driver.c
 * @brief Implementation of the Waveshare 3.97" e-paper driver
 *
 * Ported from EPD_3in97 Arduino library. Key differences:
 *  - Batched per-row SPI writes instead of byte-by-byte (avoids WDT)
 *  - eink_display_partial indexes into full frame at correct offset
 *  - 10-second timeouts on busy-wait and data loops
 *  - All public functions return esp_err_t
 *
 * Hardware notes:
 *  - SPI bus is write-only, max_transfer_sz = 4096
 *  - Display is 800x480, 1bpp = 48000 bytes; 4-gray = 96000 bytes
 *  - ESP32-C6, SPI2_HOST
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_timer.h"

#include "eink-driver.h"

static const char *TAG = "EINK";
static spi_device_handle_t eink_spi = NULL;

/**
 * @brief Macro to check an esp_err_t return and bail out on failure
 *
 * Logs the failing expression and returns the error code immediately.
 * Used instead of raw if-checks for readability.
 */
#define EINK_CHECK(x) do {                       \
    esp_err_t __err_rc = (x);                    \
    if (__err_rc != ESP_OK) {                    \
        ESP_LOGE(TAG, "%s failed: %s",           \
                 #x, esp_err_to_name(__err_rc)); \
        return __err_rc;                         \
    }                                            \
} while(0)

// Pin assignments from menuconfig
#define PIN_MOSI CONFIG_EINK_PIN_MOSI
#define PIN_SCLK CONFIG_EINK_PIN_SCLK
#define PIN_CS   CONFIG_EINK_PIN_CS
#define PIN_RST  CONFIG_EINK_PIN_RST
#define PIN_BUSY CONFIG_EINK_PIN_BUSY
#define PIN_DC   CONFIG_EINK_PIN_DC

#define SPI_HOST SPI2_HOST

/* ------------------------------------------------------------------ */
/*  Low-level SPI helpers                                             */
/* ------------------------------------------------------------------ */

/**
 * @brief Hardware reset: pulse RST low for 20 ms, then high for 20 ms
 *
 * Matches EPD_3IN97_Reset() in the Arduino library (20 ms high, 2 ms low,
 * 20 ms high). The reset pulse clears the controller's internal state,
 * including both frame buffers (0x24 and 0x26).
 */
static inline void eink_reset(void){
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
}

/**
 * @brief Read the BUSY pin level
 * @return true if display is busy (high), false if idle (low)
 *
 * An e-paper display is busy during waveform application (refresh) and
 * during certain initialization steps.
 */
static inline bool eink_is_busy(void){
    return gpio_get_level(PIN_BUSY);
}

/**
 * @brief Send a single-byte command over SPI
 *
 * Sets DC low (command mode), transmits 8 bits.
 */
static esp_err_t eink_write_cmd(uint8_t cmd){
    gpio_set_level(PIN_DC, 0);

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };

    return spi_device_transmit(eink_spi, &t);
}

/**
 * @brief Send a multi-byte data payload over SPI
 *
 * Sets DC high (data mode). Batched transmission — unlike the Arduino
 * library which sends one byte at a time, this sends up to `len` bytes
 * in a single SPI transaction. Essential for performance: 480 rows at
 * 100 bytes/row = 48 transactions instead of 48000.
 *
 * @param data  pointer to the byte buffer
 * @param len   number of bytes to send
 */
static esp_err_t eink_write_data(const uint8_t *data, size_t len){
    gpio_set_level(PIN_DC, 1);

    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };

    return spi_device_transmit(eink_spi, &t);
}

/**
 * @brief Send a single data byte
 *
 * Convenience wrapper for one-byte register writes (configuration,
 * commands that need a single argument). Still uses batched SPI under
 * the hood but has the same effect as EPD_3IN97_SendData().
 */
static esp_err_t eink_write_data_single(const uint8_t data){
    gpio_set_level(PIN_DC, 1);

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };

    return spi_device_transmit(eink_spi, &t);
}

/* ------------------------------------------------------------------ */
/*  Public: eink_init                                                  */
/* ------------------------------------------------------------------ */

esp_err_t eink_init(void) {
    static bool bus_inited = false;

    esp_err_t ret;

    /* --- Configure control pins --- */
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << PIN_RST) |
                         (1ULL << PIN_DC),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&out_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "output gpio config failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }
    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << PIN_BUSY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&in_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "busy gpio config failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(PIN_RST, 1);
    gpio_set_level(PIN_DC, 1);

    /* --- SPI Bus Initialization (idempotent) --- */
    if (!bus_inited) {
        spi_bus_config_t buscfg = {
            .mosi_io_num = PIN_MOSI,
            .miso_io_num = -1,      // write-only display
            .sclk_io_num = PIN_SCLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4096,
        };
        ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
            return ret;
        }
        bus_inited = true;
    }

    /* --- SPI Device Initialization --- */
    if (eink_spi == NULL) {
        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = CONFIG_SPI_FREQUENCY * 1000 * 1000,
            .mode = 0,
            .spics_io_num = PIN_CS,
            .queue_size = 1,
            .flags = 0,
        };

        ret = spi_bus_add_device(SPI_HOST, &devcfg, &eink_spi);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_add_device failed: %s",
                     esp_err_to_name(ret));
            return ret;
        }
    }

    ESP_LOGI(TAG, "E-Ink SPI initialized/reused");
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Internal: wait for busy, turn on display                          */
/* ------------------------------------------------------------------ */

/**
 * @brief Poll the BUSY pin until it goes low (idle) or 10 s elapse
 *
 * Waits 100 ms before first check, then polls every 10 ms. Logs the
 * elapsed time on success.
 *
 * @return ESP_OK, or ESP_ERR_TIMEOUT if BUSY stays high > 10 s
 */
static esp_err_t eink_wait_busy(void){
    int64_t start = esp_timer_get_time() / 1000;
    vTaskDelay(pdMS_TO_TICKS(100));

    while (eink_is_busy()) {
        vTaskDelay(pdMS_TO_TICKS(10));

        if ((esp_timer_get_time() / 1000) - start > 10000) {
            ESP_LOGE(TAG, "BUSY timeout");
            return ESP_ERR_TIMEOUT;
        }
    }

    int64_t elapsed = (esp_timer_get_time() / 1000) - start;
    ESP_LOGI(TAG, "BUSY released after %lld.%03llds", elapsed / 1000, elapsed % 1000);
    return ESP_OK;
}

/**
 * @brief Write command 0x22 with mode byte, then 0x20, and wait for busy
 *
 * The mode byte selects the drive waveform:
 *  - 0xF7 = normal (full refresh, flickers)
 *  - 0xD7 = fast / 4-gray
 *  - 0xFF = partial (no flicker, diff-based)
 *
 * Matches EPD_3IN97_TurnOnDisplay / _Fast / _4GRAY / _Part.
 *
 * @param mode  one of 0xF7, 0xD7, 0xFF
 */
static esp_err_t eink_turn_on_display(uint8_t mode)
{
    EINK_CHECK(eink_write_cmd(0x22));
    EINK_CHECK(eink_write_data_single(mode));
    EINK_CHECK(eink_write_cmd(0x20));

    return eink_wait_busy();
}

/* ------------------------------------------------------------------ */
/*  Public: initialize functions                                       */
/* ------------------------------------------------------------------ */

esp_err_t eink_initialize(void){
    eink_reset();
    EINK_CHECK(eink_wait_busy());
    EINK_CHECK(eink_write_cmd(0x12)); // SWRESET
    EINK_CHECK(eink_wait_busy());
    EINK_CHECK(eink_write_cmd(0x18));
    EINK_CHECK(eink_write_data_single(0x80));
    EINK_CHECK(eink_write_cmd(0x0C));
    EINK_CHECK(eink_write_data((uint8_t[]){0xAE,0xC7,0xC3,0xC0,0x80},5));

    EINK_CHECK(eink_write_cmd(0x01)); // Driver output control
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)/256));
    EINK_CHECK(eink_write_data_single(0x02));

    EINK_CHECK(eink_write_cmd(0x3C)); // Border Waveform
    EINK_CHECK(eink_write_data_single(0x01));

    EINK_CHECK(eink_write_cmd(0x11)); // Data entry mode
    EINK_CHECK(eink_write_data_single(0x01));

    EINK_CHECK(eink_write_cmd(0x44)); // Set RAM X start/end
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_write_data_single((DISPLAY_WIDTH-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_WIDTH-1)/256));

    EINK_CHECK(eink_write_cmd(0x45)); // Set RAM Y start/end
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)/256));
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));

    EINK_CHECK(eink_write_cmd(0x4E));
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_write_cmd(0x4F));
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_wait_busy());
    return ESP_OK;
}

esp_err_t eink_initialize_fast(void){
    eink_reset();
    EINK_CHECK(eink_wait_busy());
    EINK_CHECK(eink_write_cmd(0x12)); // SWRESET
    EINK_CHECK(eink_wait_busy());
    EINK_CHECK(eink_write_cmd(0x0C));
    EINK_CHECK(eink_write_data((uint8_t[]){0xAE,0xC7,0xC3,0xC0,0x80},5));

    EINK_CHECK(eink_write_cmd(0x01)); // Driver output control
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)/256));
    EINK_CHECK(eink_write_data_single(0x02));

    EINK_CHECK(eink_write_cmd(0x3C)); // Border Waveform
    EINK_CHECK(eink_write_data_single(0x01));

    EINK_CHECK(eink_write_cmd(0x11)); // Data entry mode
    EINK_CHECK(eink_write_data_single(0x01));

    EINK_CHECK(eink_write_cmd(0x44)); // Set RAM X start/end
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_write_data_single((DISPLAY_WIDTH-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_WIDTH-1)/256));

    EINK_CHECK(eink_write_cmd(0x45)); // Set RAM Y start/end
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)/256));
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));

    EINK_CHECK(eink_write_cmd(0x4E));
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_write_cmd(0x4F));
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_wait_busy());

    EINK_CHECK(eink_write_cmd(0x3C)); // Border Waveform
    EINK_CHECK(eink_write_data_single(0x01));

    EINK_CHECK(eink_write_cmd(0x18));
    EINK_CHECK(eink_write_data_single(0x80));

    EINK_CHECK(eink_write_cmd(0x1A)); // Fast update (~1.5 s)
    EINK_CHECK(eink_write_data_single(0x6A));
    return ESP_OK;
}

esp_err_t eink_initialize_gray(void){
    eink_reset();
    EINK_CHECK(eink_wait_busy());
    EINK_CHECK(eink_write_cmd(0x12)); // SWRESET
    EINK_CHECK(eink_wait_busy());
    EINK_CHECK(eink_write_cmd(0x0C));
    EINK_CHECK(eink_write_data((uint8_t[]){0xAE,0xC7,0xC3,0xC0,0x80},5));

    EINK_CHECK(eink_write_cmd(0x01)); // Driver output control
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)/256));
    EINK_CHECK(eink_write_data_single(0x02));

    EINK_CHECK(eink_write_cmd(0x11)); // Data entry mode
    EINK_CHECK(eink_write_data_single(0x01));

    EINK_CHECK(eink_write_cmd(0x44)); // Set RAM X start/end
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_write_data_single((DISPLAY_WIDTH-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_WIDTH-1)/256));

    EINK_CHECK(eink_write_cmd(0x45)); // Set RAM Y start/end
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)/256));
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));

    EINK_CHECK(eink_write_cmd(0x4E)); // set RAM x address count to 0
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_write_cmd(0x4F)); // set RAM y address count to 0
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_wait_busy());

    EINK_CHECK(eink_write_cmd(0x3C)); // Border Waveform
    EINK_CHECK(eink_write_data_single(0x01));

    EINK_CHECK(eink_write_cmd(0x18));
    EINK_CHECK(eink_write_data_single(0x80));

    EINK_CHECK(eink_write_cmd(0x1A)); // 4-gray
    EINK_CHECK(eink_write_data_single(0x5A));
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Internal: row-data helpers                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief Compute the number of bytes per row
 *
 * For 800 px at 1bpp: ceil(800/8) = 100 bytes/row.
 *
 * @return bytes per row
 */
static uint16_t eink_width_bytes(void) {
    return (DISPLAY_WIDTH % 8 == 0) ? (DISPLAY_WIDTH / 8) : (DISPLAY_WIDTH / 8 + 1);
}

/**
 * @brief Fill a full display buffer with a constant byte and refresh
 *
 * Sends `fill` to every pixel of the selected RAM buffer (cmd).
 * Each row is batched via eink_write_data(). 10-second timeout on the
 * SPI loop.
 *
 * @param cmd    RAM buffer selector (0x24 or 0x26)
 * @param width  bytes per row (= 100)
 * @param height number of rows (= 480)
 * @param fill   byte value to fill (0xFF = white, 0x00 = black)
 */
static esp_err_t eink_send_clear_row(uint8_t cmd, uint16_t width, uint16_t height, uint8_t fill) {
    uint8_t row[width];
    memset(row, fill, width);
    EINK_CHECK(eink_write_cmd(cmd));
    int64_t start = esp_timer_get_time() / 1000;
    for (uint16_t j = 0; j < height; j++) {
        EINK_CHECK(eink_write_data(row, width));
        if ((esp_timer_get_time() / 1000) - start > 10000) {
            ESP_LOGE(TAG, "send_clear_row timeout");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_OK;
}

/**
 * @brief Restore window registers to full-screen range
 *
 * Without this, a subsequent eink_clear() or eink_display() after
 * eink_display_partial() would only write to the partial window.
 * Sets 0x44/0x45 to 0..DISPLAY_WIDTH-1 / 0..DISPLAY_HEIGHT-1 and
 * resets the address counters to 0.
 */
static esp_err_t eink_reset_window(void) {
    EINK_CHECK(eink_write_cmd(0x44));
    EINK_CHECK(eink_write_data((uint8_t[]){0x00, 0x00}, 2));
    EINK_CHECK(eink_write_data_single((DISPLAY_WIDTH - 1) % 256));
    EINK_CHECK(eink_write_data_single((DISPLAY_WIDTH - 1) / 256));

    EINK_CHECK(eink_write_cmd(0x45));
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT - 1) % 256));
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT - 1) / 256));
    EINK_CHECK(eink_write_data((uint8_t[]){0x00, 0x00}, 2));

    EINK_CHECK(eink_write_cmd(0x4E));
    EINK_CHECK(eink_write_data((uint8_t[]){0x00, 0x00}, 2));
    EINK_CHECK(eink_write_cmd(0x4F));
    EINK_CHECK(eink_write_data((uint8_t[]){0x00, 0x00}, 2));
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Public: clear                                                      */
/* ------------------------------------------------------------------ */

esp_err_t eink_clear(void){
    uint16_t w = eink_width_bytes();
    EINK_CHECK(eink_reset_window());
    EINK_CHECK(eink_send_clear_row(0x24, w, DISPLAY_HEIGHT, 0xFF));
    EINK_CHECK(eink_send_clear_row(0x26, w, DISPLAY_HEIGHT, 0xFF));
    EINK_CHECK(eink_turn_on_display(0xF7));
    return ESP_OK;
}

esp_err_t eink_clear_black(void){
    uint16_t w = eink_width_bytes();
    EINK_CHECK(eink_reset_window());
    EINK_CHECK(eink_send_clear_row(0x24, w, DISPLAY_HEIGHT, 0x00));
    EINK_CHECK(eink_send_clear_row(0x26, w, DISPLAY_HEIGHT, 0x00));
    EINK_CHECK(eink_turn_on_display(0xF7));
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Internal: image upload helper                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief Send a full image to a RAM buffer, one row at a time
 *
 * Batches data per row (100 bytes) to stay under max_transfer_sz = 4096.
 * 10-second timeout on the SPI loop.
 *
 * @param cmd    RAM buffer (0x24 or 0x26)
 * @param image  48000-byte 1bpp buffer
 * @param width  bytes per row (= 100)
 * @param height number of rows (= 480)
 */
static esp_err_t eink_send_image(uint8_t cmd, const uint8_t *image, uint16_t width, uint16_t height) {
    EINK_CHECK(eink_write_cmd(cmd));
    int64_t start = esp_timer_get_time() / 1000;
    for (uint16_t j = 0; j < height; j++) {
        EINK_CHECK(eink_write_data(&image[j * width], width));
        if ((esp_timer_get_time() / 1000) - start > 10000) {
            ESP_LOGE(TAG, "send_image timeout");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Public: display functions                                          */
/* ------------------------------------------------------------------ */

esp_err_t eink_display(const uint8_t *image) {
    uint16_t w = eink_width_bytes();
    EINK_CHECK(eink_reset_window());
    EINK_CHECK(eink_send_image(0x24, image, w, DISPLAY_HEIGHT));
    EINK_CHECK(eink_turn_on_display(0xF7));
    return ESP_OK;
}

esp_err_t eink_display_base(const uint8_t *image) {
    uint16_t w = eink_width_bytes();
    EINK_CHECK(eink_reset_window());
    EINK_CHECK(eink_send_image(0x24, image, w, DISPLAY_HEIGHT));
    EINK_CHECK(eink_send_image(0x26, image, w, DISPLAY_HEIGHT));
    EINK_CHECK(eink_turn_on_display(0xF7));
    return ESP_OK;
}

esp_err_t eink_display_fast(const uint8_t *image) {
    uint16_t w = eink_width_bytes();
    EINK_CHECK(eink_reset_window());
    EINK_CHECK(eink_send_image(0x24, image, w, DISPLAY_HEIGHT));
    EINK_CHECK(eink_turn_on_display(0xD7));
    return ESP_OK;
}

esp_err_t eink_display_fast_base(const uint8_t *image) {
    uint16_t w = eink_width_bytes();
    EINK_CHECK(eink_reset_window());
    EINK_CHECK(eink_send_image(0x24, image, w, DISPLAY_HEIGHT));
    EINK_CHECK(eink_send_image(0x26, image, w, DISPLAY_HEIGHT));
    EINK_CHECK(eink_turn_on_display(0xD7));
    return ESP_OK;
}

/**
 * @brief Render a sub-image at (xstart, ystart) on a white background
 *
 * The caller provides a full 48000-byte image; the function extracts
 * the window region and sets non-window pixels to 0xFF (white).
 * Loops pixel-by-pixel per row (no batch optimization possible here
 * due to the blend logic).
 */
esp_err_t eink_display_window(const uint8_t *image, uint16_t xstart, uint16_t ystart, uint16_t img_w, uint16_t img_h) {
    uint16_t w = eink_width_bytes();
    uint8_t row[w];
    EINK_CHECK(eink_write_cmd(0x24));
    for (uint16_t i = 0; i < DISPLAY_HEIGHT; i++) {
        for (uint16_t j = 0; j < w; j++) {
            if (i < img_h + ystart && i >= ystart && j < (img_w + xstart) / 8 && j >= xstart / 8)
                row[j] = image[(j - xstart / 8) + (img_w / 8 * (i - ystart))];
            else
                row[j] = 0xFF;
        }
        EINK_CHECK(eink_write_data(row, w));
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    EINK_CHECK(eink_turn_on_display(0xF7));
    return ESP_OK;
}

/**
 * @brief Same as eink_display_window but writes both 0x24 and 0x26
 */
esp_err_t eink_display_window_base(const uint8_t *image, uint16_t xstart, uint16_t ystart, uint16_t img_w, uint16_t img_h) {
    uint16_t w = eink_width_bytes();
    uint8_t row[w];
    for (int pass = 0; pass < 2; pass++) {
        EINK_CHECK(eink_write_cmd(pass == 0 ? 0x24 : 0x26));
        for (uint16_t i = 0; i < DISPLAY_HEIGHT; i++) {
            for (uint16_t j = 0; j < w; j++) {
                if (i < img_h + ystart && i >= ystart && j < (img_w + xstart) / 8 && j >= xstart / 8)
                    row[j] = image[(j - xstart / 8) + (img_w / 8 * (i - ystart))];
                else
                    row[j] = 0xFF;
            }
            EINK_CHECK(eink_write_data(row, w));
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    EINK_CHECK(eink_turn_on_display(0xF7));
    return ESP_OK;
}

/**
 * @brief Partial refresh of a rectangular window
 *
 * Updates only the selected window using the no-flicker partial waveform
 * (0xFF). Resets the controller, sets window registers, then writes the
 * window data to 0x24.
 *
 * Diff from Arduino: this indexes into the full 48000-byte image at
 * `image[y * w_full + Xstart]` — the original reads `Image[i]` linearly
 * from offset 0. The caller passes a full-frame image, and the function
 * extracts the correct row/column data for the window.
 *
 * X alignment: the hardware requires X addresses to be byte-aligned
 * (8-pixel granularity). The function rounds Xstart down and Xend up
 * to the nearest byte boundary.
 *
 * Note: partial refresh blanks non-updated areas momentarily. This is
 * inherent to the display controller — after eink_reset(), the internal
 * "old" buffer (0x26) is lost. A full refresh is recommended after
 * several partial updates to prevent ghosting.
 *
 * @param image  full 48000-byte 1bpp buffer
 * @param Xstart pixel X of the window origin
 * @param Ystart pixel Y of the window origin
 * @param Xend   pixel X of the window end (exclusive)
 * @param Yend   pixel Y of the window end (exclusive)
 */
esp_err_t eink_display_partial(const uint8_t *image, uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend) {
    if (Xend <= Xstart || Yend <= Ystart) {
        ESP_LOGE(TAG, "invalid partial: Xend<=Xstart or Yend<=Ystart");
        return ESP_ERR_INVALID_ARG;
    }
    // Align X to byte boundaries (8-pixel hardware requirement)
    if ((Xstart % 8 + Xend % 8 == 8 && Xstart % 8 > Xend % 8) || Xstart % 8 + Xend % 8 == 0 || (Xend - Xstart) % 8 == 0) {
        Xstart = Xstart / 8;
        Xend = Xend / 8;
    } else {
        Xstart = Xstart / 8;
        Xend = Xend % 8 == 0 ? Xend / 8 : Xend / 8 + 1;
    }

    uint16_t width = Xend - Xstart;
    Xend -= 1;
    Yend -= 1;

    eink_reset();

    EINK_CHECK(eink_write_cmd(0x18));
    EINK_CHECK(eink_write_data_single(0x80));

    EINK_CHECK(eink_write_cmd(0x3C));
    EINK_CHECK(eink_write_data_single(0x80));

    EINK_CHECK(eink_write_cmd(0x44));
    EINK_CHECK(eink_write_data_single((Xstart * 8) & 0xFF));
    EINK_CHECK(eink_write_data_single(((Xstart * 8) >> 8) & 0xFF));
    EINK_CHECK(eink_write_data_single((Xend * 8) & 0xFF));
    EINK_CHECK(eink_write_data_single(((Xend * 8) >> 8) & 0xFF));

    EINK_CHECK(eink_write_cmd(0x45));
    EINK_CHECK(eink_write_data_single(Ystart & 0xFF));
    EINK_CHECK(eink_write_data_single((Ystart >> 8) & 0xFF));
    EINK_CHECK(eink_write_data_single(Yend & 0xFF));
    EINK_CHECK(eink_write_data_single((Yend >> 8) & 0xFF));

    EINK_CHECK(eink_write_cmd(0x4E));
    EINK_CHECK(eink_write_data_single((Xstart * 8) & 0xFF));
    EINK_CHECK(eink_write_data_single(((Xstart * 8) >> 8) & 0xFF));

    EINK_CHECK(eink_write_cmd(0x4F));
    EINK_CHECK(eink_write_data_single(Ystart & 0xFF));
    EINK_CHECK(eink_write_data_single((Ystart >> 8) & 0xFF));

    EINK_CHECK(eink_write_cmd(0x24));
    uint16_t w_full = eink_width_bytes();
    for (uint16_t y = Ystart; y <= Yend; y++) {
        EINK_CHECK(eink_write_data(&image[y * w_full + Xstart], width));
    }

    EINK_CHECK(eink_turn_on_display(0xFF));
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  4-gray conversion and display                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief Convert 4-gray packed data to a single pass of 0x24/0x26 row data
 *
 * Input: 96000 bytes, 2-bit per pixel, 2 bytes per 8-pixel group.
 *   Byte 0 = pixels 0-3, Byte 1 = pixels 4-7.
 *   Gray codes: 11=white, 10=light gray, 01=dark gray, 00=black.
 *
 * Pass 0 produces data for 0x24, pass 1 for 0x26.
 * The conversion splits each 2-bit gray value across the two buffers
 * to create the 4-gray optical effect.
 *
 * Matches EPD_3IN97_Display_4Gray() in the Arduino library.
 *
 * @param image 96000-byte 4-gray buffer
 * @param row   output row buffer (100 bytes)
 * @param w     bytes per row
 * @param base  byte offset into image for the current row
 * @param pass  0 = 0x24, 1 = 0x26
 */
static void gray_convert_pass(const uint8_t *image, uint8_t *row, uint16_t w, uint32_t base, int pass) {
    for (uint16_t x = 0; x < w; x++) {
        uint32_t idx = base + x;
        uint8_t t1a = image[idx * 2];
        uint8_t t1b = image[idx * 2 + 1];
        uint8_t t3 = 0;
        for (int j = 0; j < 2; j++) {
            uint8_t t1 = (j == 0) ? t1a : t1b;
            for (int k = 0; k < 2; k++) {
                uint8_t t2 = t1 & 0xC0;
                if (pass == 0) {
                    if (t2 == 0xC0)      t3 |= 0x00;
                    else if (t2 == 0x00)  t3 |= 0x01;
                    else if (t2 == 0x80)  t3 |= 0x01;
                    else                  t3 |= 0x00;
                } else {
                    if (t2 == 0xC0)      t3 |= 0x00;
                    else if (t2 == 0x00)  t3 |= 0x01;
                    else if (t2 == 0x80)  t3 |= 0x00;
                    else                  t3 |= 0x01;
                }
                t3 <<= 1;
                t1 <<= 2;
                t2 = t1 & 0xC0;
                if (pass == 0) {
                    if (t2 == 0xC0)      t3 |= 0x00;
                    else if (t2 == 0x00)  t3 |= 0x01;
                    else if (t2 == 0x80)  t3 |= 0x01;
                    else                  t3 |= 0x00;
                } else {
                    if (t2 == 0xC0)      t3 |= 0x00;
                    else if (t2 == 0x00)  t3 |= 0x01;
                    else if (t2 == 0x80)  t3 |= 0x00;
                    else                  t3 |= 0x01;
                }
                if (j != 1 || k != 1)
                    t3 <<= 1;
                t1 <<= 2;
            }
        }
        row[x] = t3;
    }
}

esp_err_t eink_display_4gray(const uint8_t *image) {
    uint16_t w = eink_width_bytes();
    uint8_t row[w];
    EINK_CHECK(eink_reset_window());
    for (int pass = 0; pass < 2; pass++) {
        EINK_CHECK(eink_write_cmd(pass == 0 ? 0x24 : 0x26));
        for (uint16_t y = 0; y < DISPLAY_HEIGHT; y++) {
            gray_convert_pass(image, row, w, y * w, pass);
            EINK_CHECK(eink_write_data(row, w));
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    EINK_CHECK(eink_turn_on_display(0xD7));
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Public: sleep                                                      */
/* ------------------------------------------------------------------ */

esp_err_t eink_sleep(void) {
    EINK_CHECK(eink_write_cmd(0x10));
    EINK_CHECK(eink_write_data_single(0x01));
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}
