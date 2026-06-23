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

#define EINK_CHECK(x) do {                       \
    esp_err_t __err_rc = (x);                    \
    if (__err_rc != ESP_OK) {                    \
        ESP_LOGE(TAG, "%s failed: %s",           \
                 #x, esp_err_to_name(__err_rc)); \
        return __err_rc;                         \
    }                                            \
} while(0)

//Pin Mapping (From Menuconfig)
#define PIN_MOSI CONFIG_EINK_PIN_MOSI
#define PIN_SCLK CONFIG_EINK_PIN_SCLK
#define PIN_CS   CONFIG_EINK_PIN_CS
#define PIN_RST  CONFIG_EINK_PIN_RST
#define PIN_BUSY CONFIG_EINK_PIN_BUSY
#define PIN_DC   CONFIG_EINK_PIN_DC

#define SPI_HOST SPI2_HOST

/*
 * Helper Functions
 */

static inline void eink_reset(void){
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
}

static inline bool eink_is_busy(void){
    return gpio_get_level(PIN_BUSY);
}

static esp_err_t eink_write_cmd(uint8_t cmd){

    gpio_set_level(PIN_DC, 0);

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };

    return spi_device_transmit(eink_spi, &t);
}

static esp_err_t eink_write_data(const uint8_t *data, size_t len){
    gpio_set_level(PIN_DC, 1);

    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };

    return spi_device_transmit(eink_spi, &t);
}

static esp_err_t eink_write_data_single(const uint8_t data){
    gpio_set_level(PIN_DC, 1);

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };

    return spi_device_transmit(eink_spi, &t);
}




esp_err_t eink_init(void) {
    static bool bus_inited = false;   // tracks if SPI bus was initialized

    esp_err_t ret;

    /* --- Configure control pins (always needed) --- */
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
    /* Default GPIO states */
    gpio_set_level(PIN_RST, 1);
    gpio_set_level(PIN_DC, 1);

    /* --- SPI Bus Initialization (idempotent) --- */
    if (!bus_inited) {
        spi_bus_config_t buscfg = {
            .mosi_io_num = PIN_MOSI,
            .miso_io_num = -1,      // write-only
            .sclk_io_num = PIN_SCLK,

            .quadwp_io_num = -1,
            .quadhd_io_num = -1,

            .max_transfer_sz = 4096,
        };
        ESP_LOGI(TAG, "SPI_HOST=%d", SPI_HOST);
        ESP_LOGI(TAG, "SPI2_HOST=%d", SPI2_HOST);
        ESP_LOGI(TAG, "SPI_DMA_CH_AUTO=%d", SPI_DMA_CH_AUTO);
        ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
            return ret;
        }
        bus_inited = true;  // mark bus as initialized
    }

    /* --- SPI Device Initialization (recreate if NULL) --- */
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

/*
 * eink_turn_on_display(0xF7); // normal
 * eink_turn_on_display(0xD7); // fast / 4-gray
 * eink_turn_on_display(0xFF); // partial
 *
 * */
static esp_err_t eink_turn_on_display(uint8_t mode)
{
    EINK_CHECK(eink_write_cmd(0x22));
    EINK_CHECK(eink_write_data_single(mode));
    EINK_CHECK(eink_write_cmd(0x20));

    return eink_wait_busy();
}

/*
 *init functions 
 */

esp_err_t eink_initialize(void){
    eink_reset();
    EINK_CHECK(eink_wait_busy());
    EINK_CHECK(eink_write_cmd(0x12)); //Notes as SWRESET
    EINK_CHECK(eink_wait_busy());
    EINK_CHECK(eink_write_cmd(0x18));
    EINK_CHECK(eink_write_data_single(0x80));
    EINK_CHECK(eink_write_cmd(0x0C));
    EINK_CHECK(eink_write_data((uint8_t[]){0xAE,0xC7,0xC3,0xC0,0x80},5));

    EINK_CHECK(eink_write_cmd(0x01)); //Driver output control
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)/256));
    EINK_CHECK(eink_write_data_single(0x02));

    EINK_CHECK(eink_write_cmd(0x3C)); //Border Waveform
    EINK_CHECK(eink_write_data_single(0x01));

    EINK_CHECK(eink_write_cmd(0x11)); //Data enty mode
    EINK_CHECK(eink_write_data_single(0x01));

    EINK_CHECK(eink_write_cmd(0x44)); //Set Ram-X address start/end location
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_write_data_single((DISPLAY_WIDTH-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_WIDTH-1)/256));

    EINK_CHECK(eink_write_cmd(0x45)); //Set Ram-Y address start/end location
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
    EINK_CHECK(eink_write_cmd(0x12)); //Notes as SWRESET
    EINK_CHECK(eink_wait_busy());
    EINK_CHECK(eink_write_cmd(0x0C));
    EINK_CHECK(eink_write_data((uint8_t[]){0xAE,0xC7,0xC3,0xC0,0x80},5));

    EINK_CHECK(eink_write_cmd(0x01)); //Driver output control
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)/256));
    EINK_CHECK(eink_write_data_single(0x02));

    EINK_CHECK(eink_write_cmd(0x3C)); //Border Waveform
    EINK_CHECK(eink_write_data_single(0x01));

    EINK_CHECK(eink_write_cmd(0x11)); //Data enty mode
    EINK_CHECK(eink_write_data_single(0x01));

    EINK_CHECK(eink_write_cmd(0x44)); //Set Ram-X address start/end location
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_write_data_single((DISPLAY_WIDTH-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_WIDTH-1)/256));

    EINK_CHECK(eink_write_cmd(0x45)); //Set Ram-Y address start/end location
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)/256));
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));

    EINK_CHECK(eink_write_cmd(0x4E));
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_write_cmd(0x4F));
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_wait_busy());


    EINK_CHECK(eink_write_cmd(0x3C)); //Border Waveform
    EINK_CHECK(eink_write_data_single(0x01));

    EINK_CHECK(eink_write_cmd(0x18));
    EINK_CHECK(eink_write_data_single(0x80));

    EINK_CHECK(eink_write_cmd(0x1A)); //Fast(1.5S?)
    EINK_CHECK(eink_write_data_single(0x6A));
    return ESP_OK;
}

esp_err_t eink_initialize_gray(void){
    eink_reset();
    EINK_CHECK(eink_wait_busy());
    EINK_CHECK(eink_write_cmd(0x12)); //Notes as SWRESET
    EINK_CHECK(eink_wait_busy());
    EINK_CHECK(eink_write_cmd(0x0C));
    EINK_CHECK(eink_write_data((uint8_t[]){0xAE,0xC7,0xC3,0xC0,0x80},5));

    EINK_CHECK(eink_write_cmd(0x01)); //Driver output control
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)/256));
    EINK_CHECK(eink_write_data_single(0x02));


    EINK_CHECK(eink_write_cmd(0x11)); //Data enty mode
    EINK_CHECK(eink_write_data_single(0x01));

    EINK_CHECK(eink_write_cmd(0x44)); //Set Ram-X address start/end location
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_write_data_single((DISPLAY_WIDTH-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_WIDTH-1)/256));

    EINK_CHECK(eink_write_cmd(0x45)); //Set Ram-Y address start/end location
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)%256));
    EINK_CHECK(eink_write_data_single((DISPLAY_HEIGHT-1)/256));
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));

    EINK_CHECK(eink_write_cmd(0x4E)); //set Ram x address count to 0
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_write_cmd(0x4F)); //set Ram y address count to 0x199
    EINK_CHECK(eink_write_data((uint8_t[]){0x00,0x00},2));
    EINK_CHECK(eink_wait_busy());

    EINK_CHECK(eink_write_cmd(0x3C)); //Border Waveform
    EINK_CHECK(eink_write_data_single(0x01));

    EINK_CHECK(eink_write_cmd(0x18));
    EINK_CHECK(eink_write_data_single(0x80));

    EINK_CHECK(eink_write_cmd(0x1A)); //4Gray
    EINK_CHECK(eink_write_data_single(0x5A));
    return ESP_OK;

}

/*Clear screen
 *
 * */

esp_err_t eink_clear(void){
    uint16_t width ,height;
    width = (DISPLAY_WIDTH%8==0)?(DISPLAY_WIDTH/8): (DISPLAY_WIDTH/8+1);
    height = DISPLAY_HEIGHT;
    EINK_CHECK(eink_write_cmd(0x24));
    for(uint16_t j=0;j<height;j++){
        for(uint16_t i=0;i<width;i++){
            EINK_CHECK(eink_write_data_single(0xFF));
        }
       vTaskDelay(pdMS_TO_TICKS(1)); 
    }
    EINK_CHECK(eink_write_cmd(0x26)); //Vendor does this and i dont see in docs bruh
    for(uint16_t j=0;j<height;j++){
        for(uint16_t i=0;i<width;i++){
            EINK_CHECK(eink_write_data_single(0xFF));
        }
       vTaskDelay(pdMS_TO_TICKS(1)); 
    }   
    EINK_CHECK(eink_turn_on_display(0xF7));
    return ESP_OK;
}
esp_err_t eink_clear_black(void){
    uint16_t width ,height;
    width = (DISPLAY_WIDTH%8==0)?(DISPLAY_WIDTH/8): (DISPLAY_WIDTH/8+1);
    height = DISPLAY_HEIGHT;
    EINK_CHECK(eink_write_cmd(0x24));
    for(uint16_t j=0;j<height;j++){
        for(uint16_t i=0;i<width;i++){
            EINK_CHECK(eink_write_data_single(0x00));
        }
       vTaskDelay(pdMS_TO_TICKS(1)); 
    }
    EINK_CHECK(eink_write_cmd(0x26));
    for(uint16_t j=0;j<height;j++){
        for(uint16_t i=0;i<width;i++){
            EINK_CHECK(eink_write_data_single(0x00));
        }
       vTaskDelay(pdMS_TO_TICKS(1)); 
    }   
    EINK_CHECK(eink_turn_on_display(0xF7));
    return ESP_OK;
}
