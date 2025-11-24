// Script for contact detection using an ESP32
const int hotwirePin = 25;   

bool timerRunning = false;
unsigned long startTime = 0;
unsigned long lastContactTime = 0;
int touchCounter = 0;

void setup() {
  Serial.begin(115200);              
  pinMode(hotwirePin, INPUT_PULLUP); 
}

void loop() {

  int state = digitalRead(hotwirePin);
  unsigned long now = millis();

  if (state == LOW) {  
    // Lockout of 0.5 seconds
    if (!timerRunning && (now - lastContactTime > 500)) {
      timerRunning = true;
      startTime = now;
      lastContactTime = now;

      touchCounter++;
      Serial.println("Contact!");
      Serial.printf("Number of contacts: %d\n", touchCounter);
    }
  } 
  else {
    timerRunning = false;
  }

  delay(10);
}
