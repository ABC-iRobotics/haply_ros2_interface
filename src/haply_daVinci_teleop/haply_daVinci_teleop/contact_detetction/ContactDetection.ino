#include <CapacitiveSensor.h>

#define threshold 1000

CapacitiveSensor cs_4_2 = CapacitiveSensor(4, 2);  // 10M resistor between pins 4 & 2, pin 2 is sensor pin

// Variables to manage the timer logic
bool timerRunning = false;   // Flag to check if the timer is running
unsigned long startTime = 0; // Variable to store the start time

int touchcounter = 0;        // Variable to store number of touches
bool readyForTouch = true;   // true = set wire hot again

void setup()
{
    cs_4_2.set_CS_AutocaL_Millis(0xFFFFFFFF);  // Turn off autocalibrate on channel 1 - just as an example
    Serial.begin(9600);                        // Start serial communication
}

void loop()
{
    long sensor_value = cs_4_2.capacitiveSensor(50);  // Get the sensor value
    //Serial.println(sensor_value);
    
    // detect touch
    if (sensor_value < 20 && readyForTouch) {
        touchcounter++;
        readyForTouch = false;     
        Serial.print("Touch detected! Counter = ");
        Serial.println(touchcounter);
    }

    // reset to hot wire state
    if (sensor_value > 30 && !readyForTouch) {
        readyForTouch = true;
        Serial.println("Wire is set hot again.");
    }
    

    // Check if total1 exceeds the threshold and the timer is not already running
    if (sensor_value > threshold && !timerRunning)
    {
        timerRunning = true;                   // Start the timer
        startTime = millis();                  // Record the start time in milliseconds
    }    

    delay(100);  // Small delay to avoid flooding the serial port
}

