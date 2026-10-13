const uint8_t ldr = 27; // ADC pin on the ESP32

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile float light = 0;
unsigned long lastHeartbeat = 0;

// Inside the ISR
void IRAM_ATTR onTimer() {
  // Timer ISR: read ADC value
  portENTER_CRITICAL_ISR(&timerMux); // entering the critical section
  light = analogRead(ldr);
  Serial.print("Light level: ");
  Serial.println(light);
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  // Configure timer to interrupt every 5 second
  timer = timerBegin(1000000); // 5MHz
  timerAttachInterrupt(timer, &onTimer); //attatch interrupt to timer
  timerWrite(timer, 1000000); // set counter value on the timer for 5 second period
  timerAlarm(timer, 1000000, true, 0); // enable alarm
}

void loop() {
  // Task 2: Heartbeat task (should run every 200ms)
  if (millis() - lastHeartbeat >= 200) {
    lastHeartbeat = millis();
    Serial.println("Heartbeat");
  }
}
  