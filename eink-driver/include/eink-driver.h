#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_WIDTH 800
#define DISPLAY_HEIGHT 480

esp_err_t eink_init(void);
esp_err_t eink_test(void);
esp_err_t eink_clear(void);
esp_err_t eink_clear_black(void);
esp_err_t eink_initialize(void);
esp_err_t eink_initialize_fast(void);
esp_err_t eink_initialize_gray(void);


#ifdef __cplusplus
}
#endif
