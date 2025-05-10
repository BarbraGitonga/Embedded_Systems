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
static uint8_t timer = 1000;

// Queues to be used
static QueueHandle_t queue_1; // stores delay variables
static QueueHandle_t queue_2; // stores information on LED blink times and serial data

// Tasks to implement
static TaskHandle_t task_A; // prints Q2 data to serial and monitors serial comms, echoes serial data, updates Q2 on delay values.
static TaskHandle_t task_B; // monitors Q1 for delay var, blinks LED and keeps track of blinks to send to Q2

//Task A
void taskA(void *pvParameters){
  static char *data = NULL;
  int item;
  int c;
  int idx = 0;
  int idx_cpy = 0;
  char buffer[buf_len] ={0};
  char first_five[5];

  // listening to the serial
  while(1){
  
    memset(buffer, 0, sizeof(char));

    // Prints values from Q2 to terminal
    if(xQueueReceive(queue_2, (void *)&timer, 0) == pdTRUE){
      Serial.println(timer);
    }

    // Monitors serial.
    if(Serial.available() > 0){
      c = Serial.read();
      
      // check if character c is a newline character
      if(c != '\n'){
        buffer[idx++] = c;
      }

      //store the string from the serial
      if(c == '\n'){
        buffer[idx] = '\0'; // adding a null terminator at the end to make it a string.

        data = (char*)pvPortMalloc((idx+1) * sizeof(char)); // creates a dynamic memory for the string in buffer upto the null terminator
        configASSERT(data); // ensure not NULL
        memcpy(data, buffer, idx+1); // copy the string onto data.

        memset(buffer, 0, buf_len); // clean the buffer.
        idx_cpy = idx+1; // total length of the string
        idx = 0; // reset index

        Serial.println(data);
      }

      char delay_str[10];

      // check if "delay" is part of the data and if there is an integer that follows it.
      if(idx_cpy >= 5){
        strncpy(first_five, data, 5);
        if (strncmp(data, "delay ", 6) == 0) {
          timer = atoi(&data[6]);
          // enter delay value in Q1
          xQueueSend(queue_1, (void*)&timer, 0);
        }

        // Free data memory to prevent stack overflow.
        vPortFree(data);
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
  //Read values from Q2 to get delay value.
  xQueueReceive(queue_2, (void *) &timer, 0);
  int blink = 0;
  int interval = 100;
  char msg[] = "Blinked";

  while(1){
    // Monitors Q1 and stores values in vairable delay.

    // toggle the LED
    LED_blink(timer);
    blink++;

    // count numner of blinks and send "Blinked" if LED blinks 100 times.
    if((blink - interval) == 100){
      xQueueSend(queue_2, (void *)msg, 0);
    }
    
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Creating the queues
  queue_1 = xQueueCreate(queue_len, sizeof(int));
  queue_2 = xQueueCreate(queue_len, sizeof(char) * 10);

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
