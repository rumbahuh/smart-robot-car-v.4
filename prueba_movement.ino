// Simplified Arduino Motor Test

// Motor Driver Pins (assuming an L298N-type driver)
#define MOTOR_STBY_PIN 3
#define MOTOR_A_DIR_PIN 7
#define MOTOR_A_SPEED_PIN 5 // PWM for speed
#define MOTOR_B_DIR_PIN 8
#define MOTOR_B_SPEED_PIN 6 // PWM for speed

// Constants
const int TEST_SPEED = 150; // Speed (0-255)
const int TEST_DURATION = 1000; // Duration in milliseconds (1 second)

void setup() {
    // 1. Initialize Serial for basic communication (optional, for debugging)
    Serial.begin(9600);
    Serial.println("Motor Test Start...");

    // 2. Configure Motor Driver Pins
    pinMode(MOTOR_STBY_PIN, OUTPUT);
    pinMode(MOTOR_A_DIR_PIN, OUTPUT);
    pinMode(MOTOR_A_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_B_DIR_PIN, OUTPUT);
    pinMode(MOTOR_B_SPEED_PIN, OUTPUT);

    // 3. Enable the Motor Driver (STBY HIGH)
    digitalWrite(MOTOR_STBY_PIN, HIGH);
    
    // 4. Move the car once
    moveForward(TEST_SPEED);
    Serial.println("Moving Forward...");

    // Wait for the desired duration
    delay(TEST_DURATION);
    
    // 5. Stop the motors
    stopMotors();
    Serial.println("Stopped.");
}

void loop() {
    // The main loop is empty because the action is done once in setup()
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
