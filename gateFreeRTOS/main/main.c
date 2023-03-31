/**
 * @file main.c
 * @author Pedro, Andrea, Diana
 * @brief 
 * @version 0.1
 * @date 2023-03-27
 * 
 * @copyright Copyright (c) 2023
 * @todo finished but not debugged
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
#define GPIO_ButtonOpen 32  // OnBoard PIN 8
#define GPIO_ButtonClose 33 // OnBoard PIN 10
#define GPIO_LimitClose 25  // OnBoard PIN 12
#define GPIO_LimitOpen 12   // OnBoard PIN 4
#define GPIO_Driver1 14     // OnBoard PIN 5
#define GPIO_Driver2 27     // OnBoard PIN 6
#define BLINK_GPIO CONFIG_BLINK_GPIO

#define GPIO_In ((1ULL<<GPIO_LimitClose)) | ((1ULL<<GPIO_LimitOpen)) | ((1ULL<<GPIO_ButtonClose)) | ((1ULL<<GPIO_ButtonOpen))
#define GPIO_Out ((1ULL<<GPIO_Driver1)) | ((1ULL<<GPIO_Driver2))

//TAGS
static const char* TAG = "Main";
static const char* DebounceTAG = "Debounce";
//FIFOS
static QueueHandle_t xFIFOButtons = NULL;
//Tasks prototypes
static void vTaskDebouncing(void *pvParameters);
//General globlal prototypes
static void vAbriendo(void);
static void vCerrando(void);
static void vSinMov(void);
static void vConfigPins(void);

void app_main(void)
{
    uint32_t buttonPressed;

    xFIFOButtons = xQueueCreate(5, sizeof(int));

    if(xTaskCreate(vTaskDebouncing, "Debouncing Task Open", configMINIMAL_STACK_SIZE+1024, (void *)GPIO_ButtonOpen, tskIDLE_PRIORITY, NULL) != pdPASS)
        ESP_LOGE(TAG, "Debounce: Im not ok, give me more stack");
    if(xTaskCreate(vTaskDebouncing, "Debouncing Task Close", configMINIMAL_STACK_SIZE+1024, (void *)GPIO_ButtonClose, tskIDLE_PRIORITY, NULL) != pdPASS)
        ESP_LOGE(TAG, "Debounce: Im not ok, give me more stack");

    vConfigPins();

    while (1) 
    {
        xQueueReceive(xFIFOButtons, &buttonPressed, portMAX_DELAY); //Blocks the main thread in order to wait a value from the FIFO
        ESP_LOGI(TAG, "Received data from the FIFO in main thread");
        
        //Asking for the open button
        if (0b01 == buttonPressed && 1==gpio_get_level(GPIO_LimitOpen))
        {
            ESP_LOGI(TAG, "Setting Open Motor");
            vAbriendo();
            while(gpio_get_level(GPIO_LimitOpen) == 1)
                vTaskDelay(pdMS_TO_TICKS(50));
            vSinMov();
        }
        //Asking for the close button
        else if (0b10 == buttonPressed && 1==gpio_get_level(GPIO_LimitClose))
        {
            ESP_LOGI(TAG, "Setting Close Motor");
            vCerrando();
            while(gpio_get_level(GPIO_LimitClose) == 1)
                vTaskDelay(pdMS_TO_TICKS(50));
            vSinMov();
        }
        else
        {
            ESP_LOGI(TAG, "No move");
            vSinMov();
        }
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
    //TickType_t xTicks;
    //xTicks = xTaskGetTickCount();
    //const TickType_t xDelayTime = (5 / portTICK_PERIOD_MS);
    uint32_t highs = 0, lows = 0;
    uint32_t button = (gpio_num_t) pvParameters;
    uint32_t buffer;
    for(;;)
    {
        //Waits for a stable high level state
        do
        {
            if(gpio_get_level(button) == 0)
            {
                lows++;
                highs = 0;
            }
            else
            {
                lows = 0;
                highs++;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            //making errors
            // "xTaskDelayUntil tasks.c:1477 (( xTimeIncrement > 0U ))"
            //vTaskDelayUntil(&xTicks, xDelayTime); 
        }while(highs<5);
        //Waits for a stable low level state
        do
        {
            if(gpio_get_level(button) == 0)
            {
                lows++;
                highs = 0;
            }
            else
            {
                highs++;
                lows = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            //making errors
            // "xTaskDelayUntil tasks.c:1477 (( xTimeIncrement > 0U ))"
            //vTaskDelayUntil(&xTicks, xDelayTime); 
        }while(lows<5);

        if(button == GPIO_ButtonOpen)
            buffer = (uint32_t) 0b01;
        else if(button == GPIO_ButtonClose)
            buffer = (uint32_t) 0b10;
        else
            ESP_LOGE(DebounceTAG, "Error with the button sended");
        ESP_LOGI(DebounceTAG, "Sending data to the FIFO");
        xQueueSendToBack(xFIFOButtons, &buffer, 10);
    }
}

/**
 * @brief 
 * 
 * @param pvParameters 
 * @retval none
 */
static void vAbriendo(void)
{
    gpio_set_level(GPIO_Driver1, 0);
    gpio_set_level(GPIO_Driver2, 1);   
}

static void vCerrando(void)
{
    gpio_set_level(GPIO_Driver1, 1);
    gpio_set_level(GPIO_Driver2, 0);
}

static void vSinMov(void)
{
    gpio_set_level(GPIO_Driver1, 0);
    gpio_set_level(GPIO_Driver2, 0);
}

static void vConfigPins(void)
{
    gpio_config_t io_conf;
    //Output Config
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_Out;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    //Input Config
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_In;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}