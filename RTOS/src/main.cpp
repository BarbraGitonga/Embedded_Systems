#include <freertos/FreeRTOS.h> 
#include <freertos/task.h>     
#include <freertos/queue.h>    
#include <string.h>
#include <Arduino.h>

#if CONFIG_FREERTOS_UNICODE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

static TaskHandle_t task_1 = NULL;
static TaskHandle_t task_2 = NULL;

static int LED_delay = 1000;

void blinkLED(void *parameters){
  while(1){
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(LED_delay / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(LED_delay / portTICK_PERIOD_MS);
  }
}

void LED_control(void *parameter) {
  while(1) {
    if(Serial.available()){
      String timer = Serial.readStringUntil('\n');
      timer.trim();
      int new_delay = timer.toInt();

      if(new_delay > 0){
        LED_delay = timer.toInt();
        Serial.println("Timer updated!");
      }
      else {
        Serial.println("invalid input");
      }
    }
    vTaskDelay(1000/ portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("Enter time in ms to control the blinking of an LED");

  xTaskCreatePinnedToCore(
    blinkLED,
    "blink LED",
    1024,
    NULL,
    1,
    &task_1,
    app_cpu
  );

  xTaskCreatePinnedToCore(
    LED_control,
    "control timer",
    1024,
    NULL,
    2,
    &task_2,
    app_cpu
  );
}
void loop() {
  // put your main code here, to run repeatedly:

}
