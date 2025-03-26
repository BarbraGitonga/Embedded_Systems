#include <freertos/FreeRTOS.h> 
#include <freertos/task.h>     
#include <freertos/queue.h>    

#include <Arduino.h>

#define LED 15
#if CONFIG_FREERTOS_UNICODE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

static const int led_pin = LED_BUILTIN;

void toggleLED(void *parameter) {
  Serial.begin(115200);
  Serial.println("Running on core " + String(xPortGetCoreID()));

  while(1) {
    digitalWrite(led_pin, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void toggleLEDTwo(void *parameter) {
  Serial.begin(115200);
  Serial.println("Task 2 running on core " + String(xPortGetCoreID()));

  while(1) {
    digitalWrite(LED, HIGH);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  pinMode(led_pin, OUTPUT);
  pinMode(LED, OUTPUT);

  xTaskCreatePinnedToCore(
    toggleLED,
    "Toggle LED",
    1024,
    NULL,
    1,
    NULL,
    app_cpu
  );

  xTaskCreatePinnedToCore(
    toggleLEDTwo,
    "Toggle an external LED",
    1024,
    NULL,
    1,
    NULL,
    app_cpu
  );

}

void loop() {
  // put your main code here, to run repeatedly:
}
