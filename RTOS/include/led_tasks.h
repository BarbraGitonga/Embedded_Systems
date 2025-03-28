#ifndef LED_TASKS_H
#define LED_TASKS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define LED 15

#if CONFIG_FREERTOS_UNICODE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

extern void toggleLED(void *parameter);
extern void toggleLEDTwo(void *parameter);
extern void setupLEDTasks();

#endif // LED_TASKS_H
