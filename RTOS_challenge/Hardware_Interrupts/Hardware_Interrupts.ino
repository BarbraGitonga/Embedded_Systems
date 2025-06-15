#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

#define TIMER_FREQ_HZ 40000000  // 40MHz (workaround for core issue)

static const uint64_t timer_max_count = 40000000;  // 1 second interval
static const TickType_t task_delay = 2000 / portTICK_PERIOD_MS;

static const uint16_t adc_pin = 36;

// Globals
static hw_timer_t *timer = NULL;
static volatile int avg = 0.0f;
static SemaphoreHandle_t sem_buf_ready; // semaphore to access read or write to buffer.
static SemaphoreHandle_t mux_avg; // semphore to access the change of global variable avg.

// Buffer Information
enum {BUF_SIZE = 10};
static volatile uint16_t buffer[BUF_SIZE]; // shared buffer. circular
static volatile int head = 0; // Writing index to buffer
static volatile int tail = 0; // Reading index to buffer
static volatile int count = 0;
static const int num_writes = 10;

// Interrupt Service Routine

// Function that reads from the adc and stores value in the buffer.
void IRAM_ATTR getADCValue() {
  BaseType_t task_woken = pdFALSE;
  buffer[head] = analogRead(adc_pin);
  head = (head + 1) % BUF_SIZE;
  count++;
  if (count >= BUF_SIZE) {
    count = 0;
    xSemaphoreGiveFromISR(sem_buf_ready, &task_woken);
  }

  // Exit from ISR
  if(task_woken) {
    portYIELD_FROM_ISR();
  }
}

// Task that reads from adc buffer and calculates the avaergate of 10 samples.
void average(void *parameters){
  uint32_t sum = 0;

  while (1) {
    // take semaphore to access the buffer and read from it

    if((xSemaphoreTake(sem_buf_ready, portMAX_DELAY)) == pdTRUE){
      // read 10 values from the buffer and get the summation
      sum = 0;
      for (int j = 0; j < BUF_SIZE; j++){
        uint16_t sample = buffer[tail];
        tail = (tail + 1) % BUF_SIZE;
        sum += sample;
      }

      // critical section
      float localAvg = (float)sum / BUF_SIZE;
      xSemaphoreTake(mux_avg, portMAX_DELAY);
      avg = localAvg;
      xSemaphoreGive(mux_avg);
    }
  }
}

// reads from serial, echos back and returns the value of avg
void listener(void *params) {
  char line[16];
  size_t idx = 0;
  Serial.setTimeout(10);

  while (1) {
    if (Serial.available()) {
      char c = Serial.read();
      Serial.write(c);

      if (c == '\n' || idx >= sizeof(line) - 1) {
        line[idx] = '\0';
        if (strcmp(line, "avg") == 0) {
          xSemaphoreTake(mux_avg, portMAX_DELAY);
            Serial.printf("Average = %.2f\n", avg);
          xSemaphoreGive(mux_avg);
        }
        idx = 0;  // reset buffer
      } else if (isPrintable(c)) {
        line[idx++] = c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {
  Serial.begin(115200);

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS Hardware Interrupt Solution---");

  // Create semaphore before it is used (in task or ISR)
  sem_buf_ready = xSemaphoreCreateCounting(BUF_SIZE, 0);
  mux_avg = xSemaphoreCreateMutex();

  // Force reboot if we can't create the semaphore
  configASSERT(sem_buf_ready && mux_avg);

  xTaskCreatePinnedToCore(
    average,
    "Average",
    2048,
    NULL,
    2,
    NULL,
    app_cpu
  );

  xTaskCreatePinnedToCore(
    listener,
    "listener",
    2048,
    NULL,
    1,
    NULL,
    app_cpu
  );

  // Create and start timer
  timer = timerBegin(TIMER_FREQ_HZ);

  // Provide ISR to timer (timer, function)
  timerAttachInterrupt(timer, &getADCValue);

  // At what count should ISR trigger (timer, count, autoreload)
  timerAlarm(timer, timer_max_count, true, 0);
}

void loop() {
  // put your main code here, to run repeatedly:

}
