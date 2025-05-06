#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Only use one core
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

static TaskHandle_t task_1 = NULL;
static TaskHandle_t task_2 = NULL;

static volatile uint8_t msg_flag = 0;
static const uint8_t len = 255;
static char *msg = NULL;

// Task to read from Serial
void readSerial(void *parameters) {
    uint8_t idx = 0;
    char buf[len] = {0};
    char c;

    while (1) {
        if (Serial.available() > 0) {
            c = Serial.read();

            if (c != '\n' && idx < len - 1) {
                buf[idx++] = c;
            }

           
            if (c == '\n') {
                buf[idx] = '\0';  // Null-terminate 

                if (msg_flag == 0) {
                    // Allocate memory (include space for null terminator)
                    msg = (char *)pvPortMalloc((idx + 1) * sizeof(char));
                    configASSERT(msg);
                    memcpy(msg, buf, idx + 1); // include null terminator

                    msg_flag = 1;
                }

                // Reset buffer for next message
                memset(buf, 0, len);
                idx = 0;
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // avoid tight loop
    }
}

// Task to echo received message
void echo(void *parameters) {
    while (1) {
        if (msg_flag == 1) {
            Serial.print("Echo: ");
            Serial.println(msg);
            Serial.print("Free heap (bytes): ");
            Serial.println(xPortGetFreeHeapSize());

            vPortFree(msg);
            msg = NULL;
            msg_flag = 0;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // yield time to CPU
    }
}

void setup() {
    Serial.begin(115200);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println("\n---FreeRTOS Serial Echo---");
    Serial.println("Enter a string:");

    xTaskCreatePinnedToCore(
        readSerial,
        "Read Serial",
        1024,  
        NULL,
        1,
        &task_1,
        app_cpu
    );

    xTaskCreatePinnedToCore(
        echo,
        "Echo Message",
        1024,
        NULL,
        1,
        &task_2,
        app_cpu
    );

    vTaskDelete(NULL); // delete setup task
}

void loop() {
    // Not used
}
