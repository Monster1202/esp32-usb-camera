#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void http_test_task(void *pvParameters);

esp_err_t test_app(void);
// void FTC533_cycle(void);
// void FTC533_process(void);
#ifdef __cplusplus
}
#endif
