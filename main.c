#include <stdio.h>
#include <string.h>
#include <hardware/clocks.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "FreeRTOS.h"
#include "task.h"

void main_app(__unused void *params) {
    while(true) {
        vTaskDelay(300);
    }
}

int main( void )
{
    stdio_init_all();

    xTaskCreate(main_app, "MainThread", (1 * 1024), NULL, tskIDLE_PRIORITY, NULL);
    vTaskStartScheduler();
    for(;;)
    {
        // 无论怎样都不应该执行到这里.
    }
}