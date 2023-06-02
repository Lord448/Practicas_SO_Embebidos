/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


#define PORT                        CONFIG_EXAMPLE_PORT
#define KEEPALIVE_IDLE              CONFIG_EXAMPLE_KEEPALIVE_IDLE
#define KEEPALIVE_INTERVAL          CONFIG_EXAMPLE_KEEPALIVE_INTERVAL
#define KEEPALIVE_COUNT             CONFIG_EXAMPLE_KEEPALIVE_COUNT

#define GPIO_LED GPIO_NUM_2
#define ToInt(x) x-0x30

static const char *TAG = "TCP";
static const char *TimerTAG = "TimerTask";
static const char *CheckTAG = "CheckStop";
static const char *TimeCalcuTAG = "TimeCalculate";

static QueueHandle_t xFIFODataRX = NULL;
static QueueHandle_t xFIFOCheckStop = NULL;
static SemaphoreHandle_t xSemaphoreFinishedTime = NULL;
static SemaphoreHandle_t xSemaphoreCheckStop = NULL;
static SemaphoreHandle_t xSemaphoreStartTimerTask = NULL;
static SemaphoreHandle_t xSemaphoreKillTasks = NULL;
static TimerHandle_t xTimerForLed = NULL;
static TaskHandle_t xTaskHandlerTimer = NULL;
static TaskHandle_t xTaskHandlerCheck = NULL;

static uint32_t countsForTimer = 0;
static bool notStopped = true;

static void vTaskTimer(void *pvParameters);
static void vTaskCheckStop(void *pvParameters);
static void vSendToSocket(int sock, const char *string);
static uint32_t uTimeCalculate(uint16_t minutes, uint16_t seconds);
static void strclean(char *string);
static void vTimerCallback(TimerHandle_t xTimer);


static void do_retransmit(const int sock)
{
    int len;
    bool isData = true;
    char rx_buffer[128];

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } 
        else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
            xSemaphoreTake(xSemaphoreKillTasks, portMAX_DELAY);
            xQueueReset(xFIFOCheckStop);
            xQueueReset(xFIFODataRX);
            vTaskDelete(xTaskHandlerCheck);
            vTaskDelete(xTaskHandlerTimer);
            ESP_LOGW(TAG, "Deleting tasks");
        } 
        else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            if(len <= 5) {
                for(uint16_t i = 0; i < strlen(rx_buffer); i++) {
                    if(rx_buffer[i] == '.'){
                        xSemaphoreGive(xSemaphoreStartTimerTask);
                        xQueueReset(xFIFODataRX);
                        isData = true;
                        break;
                    }
                }
                if(isData) {
                    for(uint32_t i = 0; i <= len; i++) {
                        xQueueSendToBack(xFIFODataRX, &rx_buffer[i], 0);
                        rx_buffer[i] = '\0';
                    }
                    isData = false;
                }
                else {
                    for(uint32_t i = 0; i <= len; i++) {
                        xQueueSendToBack(xFIFOCheckStop, &rx_buffer[i], 0);
                    }
                }
            }
            else {
                ESP_LOGW(TAG, "More char than expected");
            }
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);

            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation.
            /*
            int to_write = len;
            while (to_write > 0) {
                int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }
                to_write -= written;
            }
            */
        }
    } while (len > 0);
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }
#ifdef CONFIG_EXAMPLE_IPV6
    else if (addr_family == AF_INET6) {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
        dest_addr_ip6->sin6_family = AF_INET6;
        dest_addr_ip6->sin6_port = htons(PORT);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
#ifdef CONFIG_EXAMPLE_IPV6
        else if (source_addr.ss_family == PF_INET6) {
            inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        xTaskCreate(vTaskTimer, "TaskTimer", configMINIMAL_STACK_SIZE + 2048, (void *) sock, 1, &xTaskHandlerTimer);
        xTaskCreate(vTaskCheckStop, "TaskCheckStop", configMINIMAL_STACK_SIZE + 2048, (void *)sock, 1, &xTaskHandlerCheck);

        do_retransmit(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
    gpio_set_intr_type(GPIO_LED, GPIO_INTR_DISABLE);

    xFIFODataRX = xQueueCreate(10, sizeof(char));
    xFIFOCheckStop = xQueueCreate(10, sizeof(char));
    xSemaphoreFinishedTime = xSemaphoreCreateBinary();
    xSemaphoreCheckStop = xSemaphoreCreateBinary();
    xSemaphoreStartTimerTask = xSemaphoreCreateBinary();
    xSemaphoreKillTasks = xSemaphoreCreateBinary();
    xTimerForLed = xTimerCreate("Timer for LED", pdMS_TO_TICKS(10), pdTRUE, (void *) 0, vTimerCallback);

#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, 5, NULL);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET6, 5, NULL);
#endif
}


static void vTaskTimer(void *pvParameters)
{
    const char Response[] = "Time Reached \n";
    const char Finisher = 'f';
    static char BufferRX[5];
    uint16_t Minutes = 0, Seconds = 0;
    int sock = (int) pvParameters;
    ESP_LOGI(TimerTAG, "Timer Task Created");
    for(;;)
    {
        xSemaphoreTake(xSemaphoreStartTimerTask, portMAX_DELAY);
        notStopped = true;
        xSemaphoreTake(xSemaphoreKillTasks, 10);

        for(uint16_t i = 0; uxQueueMessagesWaiting(xFIFODataRX) > 0; i++) {
            xQueueReceive(xFIFODataRX, &BufferRX[i], portMAX_DELAY);
        }

        ESP_LOGI(TimerTAG, "Buffer received %s", BufferRX);
        //Minutes = ToInt(BufferRX[0]);
        for(uint16_t i = 0; BufferRX[i] != '\0'; i++) {
            if(i == 0)
                Minutes = ToInt(BufferRX[i]);
            else if(BufferRX[i] == '.')
                continue;
            else if(BufferRX[i] >= 0x30 && BufferRX[i] <= 0x39) {
                Seconds *= 10;
                Seconds += ToInt(BufferRX[i]);
                //ESP_LOGI(TimerTAG, "Seconds going: %d, Toint: %d", Seconds, ToInt(BufferRX[i]));
            }
        }
        for(uint16_t i = 0; i < sizeof(BufferRX); i++)
            BufferRX[i] = '\0';
        ESP_LOGI(TimerTAG, "Minutes: %d", Minutes);
        ESP_LOGI(TimerTAG, "Seconds: %d", Seconds);

        countsForTimer = uTimeCalculate(Minutes, Seconds);
        gpio_set_level(GPIO_LED, 1);
        if(xTimerStart(xTimerForLed, 0) != pdPASS) {
            ESP_LOGE(TimerTAG, "Could not start the timer");
        }
        xSemaphoreGive(xSemaphoreCheckStop);
        xSemaphoreTake(xSemaphoreFinishedTime, portMAX_DELAY);
        Minutes = 0;
        Seconds = 0;
        gpio_set_level(GPIO_LED, 0);
        xQueueSendToBack(xFIFOCheckStop, &Finisher, 0);
        if(notStopped) {
            ESP_LOGI(TimerTAG, "Time reached");
            vSendToSocket(sock, Response);
        }
        xSemaphoreGive(xSemaphoreKillTasks);
    }
}

static void vTaskCheckStop(void *pvParameters)
{
    const char Response[] = "Time Stop \n";
    char DataRX[5];
    int sock = (int)pvParameters;
    ESP_LOGI(CheckTAG, "Check Task Created");
    for(;;)
    {
        xSemaphoreTake(xSemaphoreCheckStop, portMAX_DELAY);
        xSemaphoreTake(xSemaphoreKillTasks, 10);

        xQueueReceive(xFIFOCheckStop, &DataRX[0], portMAX_DELAY);
        for(uint16_t i = 1; i < sizeof(DataRX); i++)
            xQueueReceive(xFIFOCheckStop, &DataRX[i], 50);
        for(uint16_t i = 4; i < sizeof(DataRX); i++)
            DataRX[i] = '\0';


        ESP_LOGI(CheckTAG, "Data: %s", DataRX);
        if(strcmp(DataRX, "st") == 0 || strcmp(DataRX, "stop") == 0 || strcmp(DataRX, "s") == 0) {
            notStopped = false;
            xSemaphoreGive(xSemaphoreFinishedTime);
            ESP_LOGI(CheckTAG, "Time stop!");
            xQueueReset(xFIFOCheckStop);
            vSendToSocket(sock, Response);
            strclean(DataRX);
        }
        else if(DataRX[0] == 'f') {
            xSemaphoreGive(xSemaphoreKillTasks);
            xQueueReset(xFIFOCheckStop);
            strclean(DataRX);
            continue;
        }
        else {
            ESP_LOGI(CheckTAG, "Data not handled");
        }
        xSemaphoreGive(xSemaphoreKillTasks);
    }
}

static uint32_t uTimeCalculate(uint16_t Minutes, uint16_t Seconds)
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

static void vSendToSocket(int sock, const char *string)
{
    int to_write = strlen(string);
    while (to_write > 0) {
        int written = send(sock, string + (strlen(string) - to_write), to_write, 0);
        if (written < 0) {
            ESP_LOGE(TimerTAG, "Error occurred during sending: errno %d", errno);
        }
        to_write -= written;
    }
}

static void strclean(char *string)
{
    for(uint16_t i = 0; i >= strlen(string); i++)
        string[i] = '\0';
}

static void vTimerCallback(TimerHandle_t xTimer)
{
    uint32_t ulCount;
    const uint32_t ZeroCount = 0;
    configASSERT(xTimer);
    ulCount = (uint32_t) pvTimerGetTimerID(xTimer);

    //ESP_LOGI(TimerCallbackTAG, "Counts: %d", ulCount);
    if(ulCount == countsForTimer)
    {
        xSemaphoreGive(xSemaphoreFinishedTime);
        xTimerStop(xTimer, 0);
        vTimerSetTimerID(xTimer, (void*) ZeroCount);
        //ESP_LOGI(TimerCallbackTAG, "Giving binary semaphore: xSemaphoreFinishedTime");
    }
    else
    {
        ulCount++;
        vTimerSetTimerID(xTimer, (void*) ulCount);
    }
}