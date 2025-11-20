#include <CapacitiveSensor.h>

#define threshold 1000

CapacitiveSensor cs_4_2 = CapacitiveSensor(4, 2);  // 10M resistor between pins 4 & 2, pin 2 is sensor pin

// Variables to manage the timer logic
bool timerRunning = false;   // Flag to check if the timer is running
unsigned long startTime = 0; // Variable to store the start time

void setup()
{
    cs_4_2.set_CS_AutocaL_Millis(0xFFFFFFFF);  // Turn off autocalibrate on channel 1 - just as an example
    Serial.begin(9600);                        // Start serial communication
}

void loop()
{
    long total1 = cs_4_2.capacitiveSensor(50);  // Get the sensor value
    Serial.println(total1);

    // Check if total1 exceeds the threshold and the timer is not already running
    if (total1 > threshold && !timerRunning)
    {
        timerRunning = true;                   // Start the timer
        startTime = millis();                  // Record the start time in milliseconds
    }

    // Check if total1 goes below the threshold and the timer is running
    

    delay(100);  // Small delay to avoid flooding the serial port
}
