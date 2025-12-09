// Simplified Arduino Motor Test

// ultrasounds
#define TRIG_PIN 13
#define ECHO_PIN 12
// Motor Driver Pins (assuming an L298N-type driver)
#define MOTOR_STBY_PIN 3
#define MOTOR_A_DIR_PIN 7
#define MOTOR_A_SPEED_PIN 5 // PWM for speed
#define MOTOR_B_DIR_PIN 8
#define MOTOR_B_SPEED_PIN 6 // PWM for speed

const int READ_INTERVAL = 500; // Read distance every half second

// Constants
const int TEST_SPEED = 150; // Speed (0-255)
const int TEST_DURATION = 1200; // Duration in milliseconds (1 second)
const int MAX_OBSTACLE_DISTANCE = 30;
long obstacleDistance;
bool obstacleDetected = false;
bool canStart = true;

void setup() {
    // 1. Initialize Serial for basic communication (optional, for debugging)
    Serial.begin(9600);
    Serial.println("--- Ultrasonic Sensor Test Initialized ---");
    
    // Configure the sensor pins
    pinMode(TRIG_PIN, OUTPUT); // Sets the TRIG_PIN as an Output
    pinMode(ECHO_PIN, INPUT);  // Sets the ECHO_PIN as an Input

    // 2. Configure Motor Driver Pins
    pinMode(MOTOR_STBY_PIN, OUTPUT);
    pinMode(MOTOR_A_DIR_PIN, OUTPUT);
    pinMode(MOTOR_A_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_B_DIR_PIN, OUTPUT);
    pinMode(MOTOR_B_SPEED_PIN, OUTPUT);

    // 3. Enable the Motor Driver (STBY HIGH)
    digitalWrite(MOTOR_STBY_PIN, HIGH);
    
}

void loop() {
    if (canStart) {
        checkObstacle();
        
        if (!obstacleDetected) {
            moveForward(TEST_SPEED);
        } else {
            stopMotors();
            Serial.println("OBSTACLE_DETECTED:" + String(obstacleDistance));
        }
    }
    
    Serial.println("cants start o ya se movio...");
    delay(TEST_DURATION);
}

// --- Motor Control Functions ---

void moveForward(int speed) {
    // Set both motors to move in the forward direction
    digitalWrite(MOTOR_A_DIR_PIN, HIGH); 
    digitalWrite(MOTOR_B_DIR_PIN, HIGH);
    
    // Set the speed using PWM
    analogWrite(MOTOR_A_SPEED_PIN, speed);
    analogWrite(MOTOR_B_SPEED_PIN, speed);
}

void stopMotors() {
    // Set the speed to 0 to stop both motors
    analogWrite(MOTOR_A_SPEED_PIN, 0);
    analogWrite(MOTOR_B_SPEED_PIN, 0);
}

void checkObstacle() {
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
