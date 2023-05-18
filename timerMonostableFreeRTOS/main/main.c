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
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "sdkconfig.h"

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

#define KB_A        0xf0 
#define KB_B        0xf1 
#define KB_C        0xf2
#define KB_D        0xf3
#define KB_Start    0xf4
#define KB_Stop     0xf5

#define Intro  KB_Start
#define Return KB_Stop

#define ToInt(x) x-0x30

static const char *MainTag = "Main:";

int columnas[]={GPIO_COLUMNA_1,GPIO_COLUMNA_2,GPIO_COLUMNA_3,GPIO_COLUMNA_4};
int renglones[]={GPIO_RENGLON_1,GPIO_RENGLON_2,GPIO_RENGLON_3,GPIO_RENGLON_4};
static char tabla[]={KB_A,3,2,1,KB_B,6,5,4,KB_C,9,8,7,KB_D,KB_Stop,0,KB_Start};

static QueueHandle_t xFIFOTeclado = NULL;
static SemaphoreHandle_t xSemaphoreFinishedTime = NULL;
static SemaphoreHandle_t xSemaphoreCheckStop = NULL;

static void vTaskTeclado(void *pvParameters);
static void vTaskCheckStop(void *pvParameters);
static void IRAM_ATTR ReachedValueCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data);

void app_main(void)
{
    uint16_t tecla, counter = 0;
    uint16_t minutes, seconds = 0;
    char timeSelected[5];
    bool notInput = true;

    xFIFOTeclado = xQueueCreate(10, sizeof(int));
    xSemaphoreFinishedTime = xSemaphoreCreateBinary();
    xSemaphoreCheckStop = xSemaphoreCreateBinary();
    xTaskCreate(vTaskTeclado, "TareaTeclado", configMINIMAL_STACK_SIZE+1024, NULL, 1, NULL);
    xTaskCreate(vTaskCheckStop, "TaskCheckStop", configMINIMAL_STACK_SIZE+1024, NULL, 1, NULL);

    printf("Select the time: ");
    while(1) 
    {
        for(uint16_t i = 0; notInput; i++)
        {
            xQueueReceive(xFIFOTeclado, &tecla, portMAX_DELAY);
            switch(tecla)
            {
                case Intro:
                    if(timeSelected[i] == ' ')
                    {
                        timeSelected[i] = '0';
                        printf("0");
                    }
                    counter++;
                    if(counter == 2)
                    {
                        notInput = false;
                        timeSelected[i+1] = '/'; //I don't remember which was the string finisher in C
                    }
                    else 
                    {
                        printf(",");
                        timeSelected[i] = ',';
                    }
                break;
                case Return:
                    if(i != 0)
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
                    printf("%xh", tabla[tecla]);
                    timeSelected[i] = tabla[tecla];
                break;
            }
        }
        ESP_LOGI(MainTag, "%sh", timeSelected);
        //Once the time is input
        minutes = ToInt(timeSelected[0]);
        for(uint16_t i = 2; i < sizeof(timeSelected); i++)
        {
            seconds *= 10;
            seconds += ToInt(timeSelected[i]);
        }
        ESP_LOGI(MainTag, "Minutes: %d", minutes);
        ESP_LOGI(MainTag, "Seconds: %d", seconds);

        //Set the timer counter for the requested time here

        //Turn on the led
        gpio_set_level(GPIO_LED, 1);
        //Wait for the timer or the External ISR to get the semaphore
        xSemaphoreTake(xSemaphoreFinishedTime, portMAX_DELAY);
        gpio_set_level(GPIO_LED, 0);
    }
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
    printf("Esperando a que se presione una tecla\n");
    for (;;) // una tarea nunca debe retornar o salir
    {
        anterior=-1;
        actual=explora();
        //eliminacion de rebotes
        if(0<=actual) 
        { //Si hay una tecla presionada
            anterior=actual;
            vTaskDelay( 20 / portTICK_PERIOD_MS);
            actual=explora();
            if(actual<0) //si no hay una tecla presionada reinicia
                continue; 
            else if (actual!=anterior) //si la tecla es diferente reinicia
                continue; 
            vTaskDelay( 20 / portTICK_PERIOD_MS); 
            actual=explora();
            if(actual<0) //si no hay una tecla presionada reinicia
                continue; 
            else if (actual!=anterior) //si la tecla es diferente reinicia
                continue; 
            vTaskDelay( 20 / portTICK_PERIOD_MS); 
            actual=explora();
            if(actual<0) //si no hay una tecla presionada reinicia
                continue; 
            else if (actual!=anterior) //si la tecla es diferente reinicia
                continue; 
            //mete a la cola el codigo de la tecla
            xQueueSendToBack(xFIFOTeclado,&actual,0);
            for(;;) //funcion de autorepeticion
            {
                vTaskDelay( 250 /portTICK_PERIOD_MS ); // espera durante 250ms
                actual=explora();
                if(actual<0) //si no hay una tecla presionada reinicia
                    break; 
                else if (actual!=anterior) //si la tecla es diferente reinicia
                    break;
                xQueueSendToBack(xFIFOTeclado,&actual,0); //mete a la cola el codigo de la tecla
            }
        }
        vTaskDelay( 20 / portTICK_PERIOD_MS ); // espera durante 20 ms
    }
}

//Check if works
static void vTaskCheckStop(void *pvParameters)
{
    bool notFinishedCycle = true;
    uint16_t tecla = 0;

    for(;;)
    {
        xSemaphoreTake(xSemaphoreCheckStop, portMAX_DELAY);
        while(notFinishedCycle)
        {
            xQueueReceive(xFIFOTeclado, &tecla, portMAX_DELAY);
            switch (tecla)
            {
                case KB_Stop:
                    xSemaphoreGive(xSemaphoreFinishedTime);
                    notFinishedCycle = false;
                break;
                default:
                break;
            }
        }
    }
}

//@todo
//ISR Callbacks
static void IRAM_ATTR ReachedValueCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t FIFO = (QueueHandle_t) user_data;
}


//@todo
static void timerInit(void)
{
    timer_config_t config = {
        .clk_src = TIMER_SRC_CLK_DEFAULT,
        .divider = APB_CLK_FREQ / TIMER_RESOLUTION_HZ,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true,
    }
}