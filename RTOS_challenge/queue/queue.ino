#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "string.h"
#include "stdlib.h"

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

static const uint8_t buf_len = 255;
static const uint8_t queue_len = 10;
static uint32_t timer = 1000;

// Queues to be used
static QueueHandle_t queue_1; // stores delay variables
static QueueHandle_t queue_2; // stores information on LED blink times and serial data

// Tasks to implement
static TaskHandle_t task_A; // prints Q2 data to serial and monitors serial comms, echoes serial data, updates Q2 on delay values.
static TaskHandle_t task_B; // monitors Q1 for delay var, blinks LED and keeps track of blinks to send to Q2

//Task A
void taskA(void *pvParameters){
  static char *data = NULL;
  int c;
  int idx = 0;
  char buffer[buf_len] ={0};
  char msg_rec[50];

  memset(buffer, 0, buf_len);

  // listening to the serial
  while(1){
    // Receive and print messages from queue_2 (e.g., "Blinked")
    if (xQueueReceive(queue_2, msg_rec, 0) == pdTRUE){
      Serial.println(msg_rec);
    }

    // Read serial input
    if (Serial.available() > 0){
      c = Serial.read();

      if (c != '\n' && idx < buf_len - 1){
        buffer[idx++] = c;
      }
      else if (c == '\n'){
        buffer[idx] = '\0'; // Null-terminate the string

        // Echo input
        Serial.println(buffer);

        // Check if input starts with "delay "
        if (strncmp(buffer, "delay ", 6) == 0){
          char *endptr;
          int val = strtol(&buffer[6], &endptr, 10);
          if (endptr != &buffer[6]) {
            timer = val;
            xQueueSend(queue_1, &timer, 0);
          }
        }
        // Reset buffer
        idx = 0;
        memset(buffer, 0, buf_len);
      }
    }
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void LED_blink(int freq){
  digitalWrite(LED_BUILTIN, HIGH);
  delay(freq);
  digitalWrite(LED_BUILTIN, LOW);
  delay(freq);
}

void taskB(void *pvParameters){

  Serial.println("Task B running");
  //Read values from Q2 to get delay value.
  int blink = 0;
  char msg[] = "Blinked";

  while(1){
    int new_delay;
    // Monitors Q1 and stores values in vairable delay.
    if(xQueueReceive(queue_1, &new_delay, 0) == pdTRUE){
      timer = new_delay;
    }

    // toggle the LED
    LED_blink(timer);
    blink++;

    // count numner of blinks and send "Blinked" if LED blinks 100 times.
    if(blink % 100 == 0){
      xQueueSend(queue_2, (void *)msg, 0);
    }
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Creating the queues
  queue_1 = xQueueCreate(queue_len, sizeof(uint32_t)); // stores value of timer, an integer from task A.
  queue_2 = xQueueCreate(queue_len, sizeof(char) * 10); //stores "Blinked" message, a string from task B.

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("Enter new information!");
  xTaskCreatePinnedToCore(
    taskA,
    "Task A",
    4096,
    NULL,
    1,
    &task_A,
    app_cpu
  );

  xTaskCreatePinnedToCore(
    taskB,
    "Task B",
    2048,
    NULL,
    1,
    &task_B,
    app_cpu
  );
}

void loop() {
  // put your main code here, to run repeatedly:

}
