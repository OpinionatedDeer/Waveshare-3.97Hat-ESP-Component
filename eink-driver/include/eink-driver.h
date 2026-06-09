#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_WIDTH 800
#define DISPLAY_HEIGHT 480

esp_err_t eink_init(void);
esp_err_t eink_test(void);

#ifdef __cplusplus
}
#endif
