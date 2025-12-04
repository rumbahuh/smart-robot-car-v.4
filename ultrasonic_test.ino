// Arduino Ultrasonic Sensor Tester
// Measures distance using the HC-SR04 sensor and prints the result to the Serial Monitor.

// --- Pin Definitions ---
// Change these to match your sensor's wiring
#define TRIG_PIN 13 // Trigger Pin connected to Arduino Pin 13
#define ECHO_PIN 12 // Echo Pin connected to Arduino Pin 12

// --- Constants ---
// Time delay between readings (in milliseconds)
const int READ_INTERVAL = 500; // Read distance every half second

void setup() {
    // Start serial communication for debugging and output
    Serial.begin(9600);
    Serial.println("--- Ultrasonic Sensor Test Initialized ---");
    
    // Configure the sensor pins
    pinMode(TRIG_PIN, OUTPUT); // Sets the TRIG_PIN as an Output
    pinMode(ECHO_PIN, INPUT);  // Sets the ECHO_PIN as an Input
}

void loop() {
    // Get the distance in centimeters
    long distanceCm = getDistance();
    
    // Print the result to the serial monitor
    if (distanceCm > 0) {
        Serial.print("Distance: ");
        Serial.print(distanceCm);
        Serial.println(" cm");
    } else {
        // If the sensor reading is invalid (e.g., timed out)
        Serial.println("Distance: Out of range or invalid reading.");
    }
    
    // Wait for the defined interval before taking the next reading
    delay(READ_INTERVAL);
}

// Function to measure the distance in centimeters
long getDistance() {
    // 1. Clear the trigger pin by setting it LOW for a short period
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    
    // 2. Trigger the sensor: Send a 10 microsecond HIGH pulse to the trigger pin
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // 3. Read the echo pin, pulseIn() returns the duration of the sound wave travel
    long duration = pulseIn(ECHO_PIN, HIGH, 50000); // Timeout after 50ms
    
    // 4. Calculate the distance
    // Speed of sound is 0.034 cm/microsecond. 
    // The duration is for the round trip (to object and back), so we divide by 2.
    // Distance = (Time * Speed of Sound) / 2
    long distance = duration * 0.034 / 2;
    
    return distance;
}
