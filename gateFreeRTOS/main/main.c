/**
 * @file main.c
 * @author Pedro, Andrea, Diana
 * @brief 
 * @version 0.1
 * @date 2023-03-27
 * 
 * @copyright Copyright (c) 2023
 * @todo Task debounce, Task Gate, ESP_LOGE in the correct form
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

//Pinout
#define GPIO_ButtonOpen 23
#define GPIO_ButtonClose 22
#define GPIO_LimitClose 1
#define GPIO_LimitOpen 3
#define GPIO_Driver1 21
#define GPIO_Driver2 19
#define BLINK_GPIO CONFIG_BLINK_GPIO

#define GPIO_In ((1ULL<<GPIO_LimitClose)) | ((1ULL<<GPIO_LimitOpen)) 
#define GPIO_Out ((1ULL<<GPIO_ButtonClose)) | ((1ULL<<GPIO_ButtonOpen)) | ((1ULL<<GPIO_Driver1)) | ((1ULL<<GPIO_Driver2))

//TAGS
static const char* TAG = "Main";

//FIFOS
static QueueHandle_t xFIFOButtons;
//Tasks prototypes
static void vTaskDebouncing(void *pvParameters);
static void vTaskGate(void *pvParameters);

void app_main(void)
{
    xFIFOButtons = xQueueCreate(10, sizeof(int));
    if(xTaskCreate(vTaskDebouncing, "Debouncing Task", configMINIMAL_STACK_SIZE+1024, NULL, 1, NULL) != pdPASS)
        ESP_LOGE(TAG, "Debounce: Im not ok, give me more stack");
    if(xTaskCreate(vTaskGate, "Gate Task", configMINIMAL_STACK_SIZE+1024, NULL, 1, NULL) != pdPASS)
        ESP_LOGE(TAG, "Gate: Im no ok, give me more stack");

    while (1) 
    {
        
    }
}

/**
 * @brief Debounce the terminals GPIO_ButtonOpen and GPIOButtonClose, and gives the information to the Gate Task
 *        with the queue xFIFOButtons
 * 
 * @param pvParameters this task recieves null parameters
 * @retval none
 */
static void vTaskDebouncing(void *pvParameters)
{


    for(;;)
    {

    }
}

/**
 * @brief 
 * 
 * @param pvParameters 
 * @retval none
 */
static void vTaskGate(void *pvParameters)
{


    for(;;)
    {

    }
}