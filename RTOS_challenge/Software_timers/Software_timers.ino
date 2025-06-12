#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

static TimerHandle_t led_timer = NULL;
static TimerHandle_t empty_timer = NULL;
static TaskHandle_t echo = NULL;


void readSerial(void *parameters) {
    char c;

    while (1) {
        if (Serial.available() > 0) {
            c = Serial.read();
            Serial.println(c);
            digitalWrite(LED_BUILTIN, HIGH);
            xTimerReset(led_timer, portMAX_DELAY);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void myTimerCallBack(TimerHandle_t xTimer){
  if ((uint32_t)pvTimerGetTimerID(xTimer) == 0){
   digitalWrite(LED_BUILTIN, LOW);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  led_timer = xTimerCreate(
    "LED timer",
    5000 / portTICK_PERIOD_MS,
    pdFALSE,
    (void *)0,
    myTimerCallBack
  );

  xTaskCreatePinnedToCore(
      readSerial,      // Function to implement the task
      "Serial Echo",   // Name of the task
      2048,            // Stack size
      NULL,            // Task input parameter
      1,               // Priority
      &echo,           // Task handle
      app_cpu          // Core where the task should run
  );


  if(led_timer == NULL){
    Serial.println("Could not create one of the timers");
  } else{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println("Starting timers...");

    xTimerStart(led_timer, portMAX_DELAY);
  }
  vTaskDelete(NULL);
}

void loop() {
  // put your main code here, to run repeatedly:

}
