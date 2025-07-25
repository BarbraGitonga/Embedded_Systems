# Mutexes

This is a challenge from [Digikey mutexes](https://www.digikey.com/en/maker/projects/introduction-to-rtos-solution-to-part-6-freertos-mutex-example/c6e3581aa2204f1380e83a9b4c3807a6).

> Starting with the code given below, modify it to protect the task parameter (delay_arg) with a mutex. 
> With the mutex in place, the task should be able to read the parameter (parameters) into the local variable (num)
> before the calling function’s stack memory goes out of scope (the value given by delay_arg).

In RTOS, a Mutex (Mutual Exclusion) is used to control exclusive access to a shared resource, in this case
the parameter passed to the task. Like a key that lets one access a specific room.

The main purpose is to safely access the parameter `delay_arg` passed to the `blinkLED` task.
The `blinkLED` task lives on the stack of the `setup()` function.

After initaliazing the mutex 
```C
mutex = xSemaphoreCreateMutex();
```
The `blinkLED` first tries to take the mutex and if successful, it gets access to the memory holding
the passed parameter.
It copies the value to a local variable `num` then the task configures the LED pin and enters
an infinite while loop.

```
void blinkLED(void *parameters) {
  if (xSemaphoreTake(mutex, 0) == pdPASS) {
    // Copy the parameter into a local variable
    int num = *(int *)parameters;

    // Release the mutex immediately after done with shared resource
    xSemaphoreGive(mutex);

    // Print the parameter
    Serial.print("Received: ");
    Serial.println(num);

    // Configure the LED pin
    pinMode(led_pin, OUTPUT);

    // Blink forever
    while (1) {
      digitalWrite(led_pin, HIGH);
      vTaskDelay(num / portTICK_PERIOD_MS);
      digitalWrite(led_pin, LOW);
      vTaskDelay(num / portTICK_PERIOD_MS);
    }
  }
}

```
