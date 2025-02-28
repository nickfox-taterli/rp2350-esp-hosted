//
// Created by TaterLi on 25-2-8.
//

#ifndef ESP_CONFIG_H
#define ESP_CONFIG_H

#include "FreeRTOS.h"

#define H_USE_MEMPOOL 1

#define H_WIFI_TX_DATA_THROTTLE_HIGH_THRESHOLD 90
#define H_WIFI_TX_DATA_THROTTLE_LOW_THRESHOLD 60

#define H_MAX_SYNC_RPC_REQUESTS                      5
#define H_MAX_ASYNC_RPC_REQUESTS                     5

#define RPC_TASK_STACK_SIZE                          (5*1024)
#define RPC_TASK_PRIO                                23
#define DFLT_TASK_STACK_SIZE                         (5*1024)
#define DFLT_TASK_PRIO                               23

#define HOSTED_BLOCK_MAX                             portMAX_DELAY

#endif //ESP_CONFIG_H
