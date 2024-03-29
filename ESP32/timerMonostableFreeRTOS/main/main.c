/**
 * @file main.c
 * @author Pedro, Andrea, Diana
 * @brief This program is a timer controller that can start and stop the the led in the number 2
 * @version 0.1
 * @date 2023-04-04
 * @note The keyboard convention is the following:
 * 
 *          Keyboard layout            Key code
 *          |--------------------|        |----------------|
 *          |   1    2    3    A |      3 | 12  13  14  15 |
 *          |   4    5    6    B |      2 | 8   9   10  11 |
 *          |   7    8    9    C |      1 | 4   5   6   7  |
 *          | Start  0  Stop   D |      0 | 0   1   2   3  |
 *          |--------------------|        |----------------|
 *                                          0   1   2   3
 * 
 * @copyright Copyright (c) 2023
 * @todo all
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "sdkconfig.h"

//#define DEBUG

#define N_RENGLON 4
#define GPIO_RENGLON_1    32 
#define GPIO_RENGLON_2    33 
#define GPIO_RENGLON_3    25 
#define GPIO_RENGLON_4    26 

#define N_COLUMNA 4
#define GPIO_COLUMNA_1    27
#define GPIO_COLUMNA_2    14
#define GPIO_COLUMNA_3    12
#define GPIO_COLUMNA_4    19

#define GPIO_RENGLONES_SEL  ((1ULL<<GPIO_RENGLON_1) | (1ULL<<GPIO_RENGLON_2) | (1ULL<<GPIO_RENGLON_3) | (1ULL<<GPIO_RENGLON_4))
#define GPIO_COLUMNAS_SEL  ((1ULL<<GPIO_COLUMNA_1) | (1ULL<<GPIO_COLUMNA_2) | (1ULL<<GPIO_COLUMNA_3) | (1ULL<<GPIO_COLUMNA_4))

#define GPIO_LED GPIO_NUM_2

#define KB_A      'A' 
#define KB_B      'B'
#define KB_D      'D'
#define KB_Stop   'S'
#define KB_C      'C'
#define KB_Start  'T'

#define Intro  KB_Start
#define Return KB_Stop

#define ToInt(x) x-0x30

static const char *MainTag = "Main";
static const char *TimeCalcuTAG = "TimeCalculate";
static const char *TimerCallbackTAG = "TimerCallback";
static const char *CheckTag = "CheckStop";

static int columnas[]={GPIO_COLUMNA_1,GPIO_COLUMNA_2,GPIO_COLUMNA_3,GPIO_COLUMNA_4};
static int renglones[]={GPIO_RENGLON_1,GPIO_RENGLON_2,GPIO_RENGLON_3,GPIO_RENGLON_4};
static char tabla[]={'1', '2', '3', KB_A, '4', '5', '6', KB_B, '7', '8', '9', KB_C, KB_Start, '0', KB_Stop, KB_D};

static QueueHandle_t xFIFOTeclado = NULL;
static QueueHandle_t xFIFOTimer = NULL;
static SemaphoreHandle_t xSemaphoreFinishedTime = NULL;
static SemaphoreHandle_t xSemaphoreCheckStop = NULL;
static SemaphoreHandle_t xSemaphoreStartMain = NULL;
static TimerHandle_t xTimerForLed = NULL;

static uint32_t countsForTimer = 0;

static void vTaskTeclado(void *pvParameters);
static void vTaskCheckStop(void *pvParameters);
static void vTimerCallback(TimerHandle_t xTimer);
static uint32_t vTimeCalculate(uint16_t minutes, uint16_t seconds);

void app_main(void)
{
    uint16_t tecla = 0, counter = 0;
    uint16_t Minutes = 0, Seconds = 0;
    static char timeSelected[5];
    bool notInput = true;

    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
    gpio_set_intr_type(GPIO_LED, GPIO_INTR_DISABLE);
    xFIFOTeclado = xQueueCreate(10, sizeof(int));
    xFIFOTimer = xQueueCreate(10, sizeof(int));
    xSemaphoreFinishedTime = xSemaphoreCreateBinary();
    xSemaphoreCheckStop = xSemaphoreCreateBinary();
    xSemaphoreStartMain = xSemaphoreCreateBinary();
    xTimerForLed = xTimerCreate("Timer For Led", pdMS_TO_TICKS(10), pdTRUE, (void *) 0, vTimerCallback);
    xTaskCreate(vTaskTeclado, "TareaTeclado", configMINIMAL_STACK_SIZE+2048, NULL, 1, NULL);
    xTaskCreate(vTaskCheckStop, "TaskCheckStop", configMINIMAL_STACK_SIZE+1024, NULL, 1, NULL);
    xSemaphoreTake(xSemaphoreStartMain, portMAX_DELAY);
    while(1) 
    {
        ESP_LOGI(MainTag, "Select the time:");
        for(int16_t i = 0; notInput; i++)
        {
            xQueueReceive(xFIFOTeclado, &tecla, portMAX_DELAY);            
            switch(tabla[tecla])
            {
                case KB_Start:
                    if(timeSelected[i] == ' ')
                    {
                        timeSelected[i] = '0';
                        ESP_LOGI(MainTag, "0");
                    }
                    counter++;
                    if(counter == 2)
                    {
                        notInput = false;
                        timeSelected[i+1] = '\0'; //Standard string finisher
                    }
                    else 
                    {
                        ESP_LOGI(MainTag, ",");
                        timeSelected[i] = ',';
                    }
                break;
                case KB_Stop:
                    if(i > 0)
                    {   
                        i--;
                        timeSelected[i] = ' ';
                    }
                break;
                case KB_A:
                case KB_B:
                case KB_C:
                case KB_D:
                break;
                default:
                    ESP_LOGI(MainTag, "%c", tabla[tecla]);
                    
                    timeSelected[i] = tabla[tecla];
                break;
            }
        }

        //Once the time is input
        Minutes = ToInt(timeSelected[0]);

        for(uint16_t i = 2; timeSelected[i] != '\0'; i++)
        {
            Seconds *= 10;
            Seconds += ToInt(timeSelected[i]);
        }
        ESP_LOGI(MainTag, "Minutes: %d", Minutes);
        ESP_LOGI(MainTag, "Seconds: %d", Seconds);

        //Set the timer counter for the requested time here
        countsForTimer = vTimeCalculate(Minutes, Seconds);
        //Turn on the led
        gpio_set_level(GPIO_LED, 1);
        if(xTimerStart(xTimerForLed, 0) != pdPASS)
            ESP_LOGE(MainTag, "Could not start the timer");
        xSemaphoreGive(xSemaphoreCheckStop);
        //Wait for the timer or the External ISR to get the semaphore
        xSemaphoreTake(xSemaphoreFinishedTime, portMAX_DELAY);
        for(uint16_t i = 0; i > sizeof(timeSelected); i++)
            timeSelected[i] = ' ';
        xQueueReset(xFIFOTeclado);
        notInput = true;
        counter = 0;
        Minutes = 0;
        Seconds = 0;
        gpio_set_level(GPIO_LED, 0);
    }
}

static uint32_t vTimeCalculate(uint16_t Minutes, uint16_t Seconds)
{
    const uint16_t periodMS = 10;
    uint32_t countsNeeded = 0;
    uint32_t Milliseconds;

    Seconds += Minutes * 60;
    Milliseconds = Seconds * 1000;
    countsNeeded =  Milliseconds / periodMS;
    ESP_LOGI(TimeCalcuTAG, "Time in seconds received: %d", Seconds);
    ESP_LOGI(TimeCalcuTAG, "Time in counts setted: %d", (int)countsNeeded);

    return countsNeeded;
}

signed int explora()
{
    int renglon,columna,tecla;
    
    //supone que no hay una tecla presionada
    tecla=-1;
    //exploracion del teclado multiplexado
    for(columna=0;columna<N_COLUMNA;columna++){
      gpio_set_level(columnas[columna],0);
      for(renglon=0;renglon<N_RENGLON;renglon++){
          //printf("r:%d c: %d in:%d\n",renglon,columna,gpio_get_level(renglones[renglon]));
          if(gpio_get_level(renglones[renglon])==0) {
            tecla=(columna)+(renglon*N_COLUMNA);
            gpio_set_level(columnas[columna],1);
            return(tecla); //hubo una tecla presionada, regresa codigo
          }
      }
      gpio_set_level(columnas[columna],1);
    }
    return(-1); //no hubo una tecla presionada, regresa -1
}

static void vTaskTeclado(void *pvParameters)  
{
    gpio_config_t io_conf;
    //Configura los columnas como salida
    //deshabilita las interrupciones
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //Configura como salida
    io_conf.mode = GPIO_MODE_OUTPUT;
    //Mascara de bits de las terminales que se desea configurar
    io_conf.pin_bit_mask = GPIO_COLUMNAS_SEL;
    //deshabilita el modo pull-down
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE ;
    //habilita el pull-up 
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configura las GPIO con los valores indicados
    gpio_config(&io_conf);

    //Configura los renglones como entrada con pull-up

    //habilita las interrupciones con flanco de bajada
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //Configura como entradas
    io_conf.mode = GPIO_MODE_INPUT;
    //Mascara de bits de las terminales que se desea configurar
    io_conf.pin_bit_mask = GPIO_RENGLONES_SEL;
    //habilita el modo pull-down
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //deshabilita el pull-up 
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    //configura las GPIO con los valores indicados
    gpio_config(&io_conf);
    for(unsigned columna=0;columna<4;columna++)
        gpio_set_level(columnas[columna],1);
    int anterior,actual;
    xSemaphoreGive(xSemaphoreStartMain);
    xQueueReset(xFIFOTeclado);
    for (;;) // una tarea nunca debe retornar o salir
    {
        anterior=-1;
        actual=explora();
        //eliminacion de rebotes
        if(0<=actual) 
        { 
            //printf("Entre al if \n");
            //Si hay una tecla presionada
            anterior=actual;
            vTaskDelay(20/portTICK_PERIOD_MS);
            actual=explora();
            if(actual<0) //si no hay una tecla presionada reinicia
                continue; 
            else if (actual!=anterior) //si la tecla es diferente reinicia
                continue; 
            vTaskDelay(20/portTICK_PERIOD_MS); 
            actual=explora();
            if(actual<0) //si no hay una tecla presionada reinicia
                continue; 
            else if (actual!=anterior) //si la tecla es diferente reinicia
                continue; 
            vTaskDelay(20/portTICK_PERIOD_MS); 
            actual=explora();
            if(actual<0) //si no hay una tecla presionada reinicia
                continue; 
            else if (actual!=anterior) //si la tecla es diferente reinicia
                continue; 
            //mete a la cola el codigo de la tecla
            xQueueSendToBack(xFIFOTeclado,&actual,0);
            for(;;) //funcion de autorepeticion
            {
                vTaskDelay(250/portTICK_PERIOD_MS); // espera durante 250ms
                actual=explora();
                if(actual<0) //si no hay una tecla presionada reinicia
                    break; 
                else if (actual!=anterior) //si la tecla es diferente reinicia
                    break;
                xQueueSendToBack(xFIFOTeclado,&actual,0); //mete a la cola el codigo de la tecla
            }
        }
        vTaskDelay( 20/portTICK_PERIOD_MS ); // espera durante 20 ms
    }
}

static void vTaskCheckStop(void *pvParameters)
{
    bool notFinishedCycle = true;
    uint16_t tecla = 0;

    for(;;)
    {
        xSemaphoreTake(xSemaphoreCheckStop, portMAX_DELAY);
        ESP_LOGI(CheckTag, "Check Task Running");
        while(notFinishedCycle)
        {
            xQueueReceive(xFIFOTeclado, &tecla, portMAX_DELAY);
            switch (tabla[tecla])
            {
                case KB_Stop:
                    ESP_LOGI(CheckTag, "Time Interrupted");
                    xTimerStop(xTimerForLed, 0);
                    xSemaphoreGive(xSemaphoreFinishedTime);
                    notFinishedCycle = false;
                break;
                default:
                break;
            }
        }
    }
}

static void vTimerCallback(TimerHandle_t xTimer)
{
    uint32_t ulCount;
    configASSERT(xTimer);
    ulCount = (uint32_t) pvTimerGetTimerID(xTimer);

    //ESP_LOGI(TimerCallbackTAG, "Counts: %d", ulCount);
    if(ulCount == countsForTimer)
    {
        xSemaphoreGive(xSemaphoreFinishedTime);
        xTimerStop(xTimer, 0);
        //ESP_LOGI(TimerCallbackTAG, "Giving binary semaphore: xSemaphoreFinishedTime");
    }
    else
    {
        ulCount++;
        vTimerSetTimerID(xTimer, (void*) ulCount);
    }
}