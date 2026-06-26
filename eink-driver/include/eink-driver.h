/**
 * @file eink-driver.h
 * @brief Driver for Waveshare 3.97" e-paper display (SSD1680 controller)
 *
 * Ported from EPD_3in97 Arduino library. Key differences:
 *  - All functions return esp_err_t instead of void
 *  - Bulk SPI uses batched per-row writes instead of byte-by-byte
 *  - eink_display_partial indexes into a full 48000-byte image at the
 *    correct (Ystart, Xstart) offset rather than reading linearly from 0
 *  - 10-second timeout on busy-wait and data loops
 *
 * SPI bus is write-only (MOSI only, MISO unused). Pin assignments come
 * from menuconfig (CONFIG_EINK_PIN_*).
 */

#pragma once
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Display width in pixels */
#define DISPLAY_WIDTH  800

/** Display height in pixels */
#define DISPLAY_HEIGHT 480

/**
 * @brief Initialize SPI bus, GPIO pins, and register the SPI device
 *
 * Idempotent — safe to call multiple times. Bus init runs once (static
 * flag), device init runs once (checks handle pointer).
 *
 * @return ESP_OK, or GPIO/SPI error
 */
esp_err_t eink_init(void);

/**
 * @brief Reset controller and run full init sequence (normal mode)
 *
 * Pulses RST low, issues SWRESET (0x12), then configures driver output
 * control, border waveform, data entry mode, and RAM X/Y address ranges
 * for the full 800x480 frame.
 *
 * @return ESP_OK, or ESP_ERR_TIMEOUT if busy hangs
 */
esp_err_t eink_initialize(void);

/**
 * @brief Reset controller and run init sequence for fast mode (~1.5 s)
 *
 * Adds 0x1A = 0x6A and re-sets 0x3C/0x18 after the common init setup.
 * Use before eink_display_fast() or eink_display_fast_base().
 *
 * @return ESP_OK, or ESP_ERR_TIMEOUT if busy hangs
 */
esp_err_t eink_initialize_fast(void);

/**
 * @brief Reset controller and run init sequence for 4-gray mode
 *
 * Uses 0x1A = 0x5A instead of 0x6A. Use before eink_display_4gray().
 *
 * @return ESP_OK, or ESP_ERR_TIMEOUT if busy hangs
 */
esp_err_t eink_initialize_gray(void);

/**
 * @brief Fill both RAM buffers (0x24 and 0x26) with 0xFF and refresh
 *
 * Leaves the display white. Resets the window registers to full screen
 * first, so safe to call after eink_display_partial().
 *
 * @return ESP_OK, or ESP_ERR_TIMEOUT if busy hangs
 */
esp_err_t eink_clear(void);

/**
 * @brief Fill both RAM buffers (0x24 and 0x26) with 0x00 and refresh
 *
 * Leaves the display black.
 *
 * @return ESP_OK, or ESP_ERR_TIMEOUT if busy hangs
 */
esp_err_t eink_clear_black(void);

/**
 * @brief Display a full image with normal refresh
 *
 * Writes image to buffer 0x24 only. Refresh uses 0xF7 (full drive).
 * Resets window registers before writing.
 *
 * @param image 48000-byte 1bpp buffer, row-major MSB-first
 * @return ESP_OK, or ESP_ERR_TIMEOUT if busy hangs
 */
esp_err_t eink_display(const uint8_t *image);

/**
 * @brief Display a full image with normal refresh, writing both buffers
 *
 * Writes image to both 0x24 and 0x26, then refreshes with 0xF7.
 * Stores a "base" layer for subsequent partial refreshes.
 *
 * @param image 48000-byte 1bpp buffer
 * @return ESP_OK, or ESP_ERR_TIMEOUT if busy hangs
 */
esp_err_t eink_display_base(const uint8_t *image);

/**
 * @brief Display a full image with fast refresh (~1.5 s, less flicker)
 *
 * Writes to buffer 0x24 only. Refresh uses 0xD7 (fast drive).
 * Call eink_initialize_fast() first.
 *
 * @param image 48000-byte 1bpp buffer
 * @return ESP_OK, or ESP_ERR_TIMEOUT if busy hangs
 */
esp_err_t eink_display_fast(const uint8_t *image);

/**
 * @brief Display a full image with fast refresh, writing both buffers
 *
 * @param image 48000-byte 1bpp buffer
 * @return ESP_OK, or ESP_ERR_TIMEOUT if busy hangs
 */
esp_err_t eink_display_fast_base(const uint8_t *image);

/**
 * @brief Display an image window over a white background, normal refresh
 *
 * Renders the sub-image at (xstart, ystart) on a white (0xFF) background.
 * Non-window pixels are set to white.
 *
 * @param image  full 48000-byte 1bpp buffer containing the window
 * @param xstart pixel X of the window origin
 * @param ystart pixel Y of the window origin
 * @param img_w  window width in pixels
 * @param img_h  window height in pixels
 * @return ESP_OK, or ESP_ERR_TIMEOUT if busy hangs
 */
esp_err_t eink_display_window(const uint8_t *image, uint16_t xstart, uint16_t ystart, uint16_t img_w, uint16_t img_h);

/**
 * @brief Display an image window over white, writing both buffers
 *
 * Same as eink_display_window but writes both 0x24 and 0x26.
 *
 * @param image  full 48000-byte 1bpp buffer containing the window
 * @param xstart pixel X of the window origin
 * @param ystart pixel Y of the window origin
 * @param img_w  window width in pixels
 * @param img_h  window height in pixels
 * @return ESP_OK, or ESP_ERR_TIMEOUT if busy hangs
 */
esp_err_t eink_display_window_base(const uint8_t *image, uint16_t xstart, uint16_t ystart, uint16_t img_w, uint16_t img_h);

/**
 * @brief Partial refresh of a rectangular window
 *
 * Updates only the selected window using the no-flicker partial waveform
 * (0xFF). Indexes into the full-frame image at (Ystart, Xstart), unlike the
 * original Arduino library which reads linearly from offset 0.
 *
 * Diff from Arduino: reads `image[y * w_full + Xstart]` per row instead of
 * `Image[i]` linearly. The caller passes a full 48000-byte image, and the
 * function extracts the correct window data from it.
 *
 * @param image  full 48000-byte 1bpp buffer
 * @param Xstart pixel X of the window origin
 * @param Ystart pixel Y of the window origin
 * @param Xend   pixel X of the window end (exclusive)
 * @param Yend   pixel Y of the window end (exclusive)
 * @return ESP_OK, ESP_ERR_INVALID_ARG if Xend <= Xstart or Yend <= Ystart,
 *         or ESP_ERR_TIMEOUT if busy hangs
 */
esp_err_t eink_display_partial(const uint8_t *image, uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend);

/**
 * @brief Display a 4-gray image
 *
 * Image format: 96000 bytes, 2-bit per pixel packed MSB-first.
 * Gray codes: 11=white, 10=light gray, 01=dark gray, 00=black.
 * Converts to 0x24/0x26 row data via gray_convert_pass(), then refreshes
 * with 0xD7 (fast drive). Call eink_initialize_gray() first.
 *
 * @param image 96000-byte 2bpp buffer
 * @return ESP_OK, or ESP_ERR_TIMEOUT if busy hangs
 */
esp_err_t eink_display_4gray(const uint8_t *image);

/**
 * @brief Enter deep sleep mode
 *
 * Sends 0x10 = 0x01. The display retains its current image. Wake by
 * calling eink_initialize() or eink_initialize_fast().
 *
 * @return ESP_OK
 */
esp_err_t eink_sleep(void);

#ifdef __cplusplus
}
#endif
