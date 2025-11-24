const int hotwirePin = 2;

bool timerRunning = false;
unsigned long startTime = 0;
unsigned long lastContactTime = 0;    
int touchCounter = 0;

void setup() {
  Serial.begin(9600);
  pinMode(hotwirePin, INPUT_PULLUP);
}

void loop() {

  int state = digitalRead(hotwirePin);
  unsigned long now = millis();

  if (state == LOW) {  
    // Check 0.5s lockout
    if (!timerRunning && (now - lastContactTime > 500)) {
      timerRunning = true;
      startTime = now;
      lastContactTime = now;     

      touchCounter++;
      Serial.println("Contact!");
      Serial.print("Number of contacts: ");
      Serial.println(touchCounter);
    }
  }  

  else {               
    timerRunning = false;
  }

  delay(5);
}
