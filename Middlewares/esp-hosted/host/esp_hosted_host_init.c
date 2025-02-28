/*
* SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
*
* SPDX-License-Identifier: Apache-2.0
*/

#include "esp_wrapper.h"
#include "esp_hosted_api.h"

static const char TAG[] = "host_init";

//ESP_SYSTEM_INIT_FN(esp_hosted_host_init, BIT(0), 120)
void esp_hosted_host_init(void)
{
	ESP_ERROR_CHECK(esp_hosted_init());
}

void esp_hosted_host_deinit(void)
{
	ESP_LOGI(TAG, "ESP Hosted deinit");
	esp_hosted_deinit();
}
