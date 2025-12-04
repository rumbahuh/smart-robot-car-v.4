// Test Sketch for Basic Motor Movements (Forward, Left, Right)

// --- PIN DEFINITIONS (Adjust these if your wiring changes) ---
#define MOTOR_STBY_PIN 3
#define MOTOR_A_DIR_PIN 7      // Motor A (e.g., Left Motor) Direction Pin
#define MOTOR_A_SPEED_PIN 5    // Motor A PWM Speed Pin
#define MOTOR_B_DIR_PIN 8      // Motor B (e.g., Right Motor) Direction Pin
#define MOTOR_B_SPEED_PIN 6    // Motor B PWM Speed Pin

// --- CONSTANTS ---
const int FORWARD_SPEED = 150; // Speed for moving straight (0-255)
const int TURN_SPEED = 100;    // Speed for turning (0-255)
const int MOVEMENT_DURATION = 1500; // Duration for each test movement in milliseconds (1.5 seconds)
const int STOP_DURATION = 1000;     // Duration for pauses between tests in milliseconds (1 second)

void setup() {
    Serial.begin(9600);
    Serial.println("--- Motor Function Test Start ---");
    
    // 1. Configure Motor Driver Pins
    pinMode(MOTOR_STBY_PIN, OUTPUT);
    pinMode(MOTOR_A_DIR_PIN, OUTPUT);
    pinMode(MOTOR_A_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_B_DIR_PIN, OUTPUT);
    pinMode(MOTOR_B_SPEED_PIN, OUTPUT);
    
    // 2. Enable the Motor Driver (STBY HIGH)
    digitalWrite(MOTOR_STBY_PIN, HIGH);
    
    // 3. Initial Stop
    stopMotors();
    delay(1000); // Wait for initialization

    // --- TEST SEQUENCE ---

    // Test 1: Move Forward
    Serial.println("TEST 1: Moving Forward...");
    moveForward(FORWARD_SPEED);
    delay(MOVEMENT_DURATION);
    stopMotors();
    Serial.println("Stopped.");
    delay(STOP_DURATION);

    // Test 2: Turn Left (Right Wheel Forward, Left Wheel Backward or Slower)
    Serial.println("TEST 2: Turning Left...");
    turnLeft();
    delay(MOVEMENT_DURATION);
    stopMotors();
    Serial.println("Stopped.");
    delay(STOP_DURATION);

    // Test 3: Turn Right (Left Wheel Forward, Right Wheel Backward or Slower)
    Serial.println("TEST 3: Turning Right...");
    turnRight();
    delay(MOVEMENT_DURATION);
    stopMotors();
    Serial.println("Stopped.");
    delay(STOP_DURATION);

    Serial.println("--- Test Sequence Complete ---");
}

void loop() {
    // This loop remains empty as the full test sequence runs once in setup()
}

// --- Motor Control Functions ---

/**
 * Moves the robot forward at the specified speed.
 * Assumes: HIGH on DIR pins means forward.
 * @param speed PWM value (0-255).
 */
void moveForward(int speed) {
    // Motor A (Left) Forward
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);
    analogWrite(MOTOR_A_SPEED_PIN, speed);
    
    // Motor B (Right) Forward
    digitalWrite(MOTOR_B_DIR_PIN, HIGH);
    analogWrite(MOTOR_B_SPEED_PIN, speed);
}

/**
 * Turns the robot left.
 * This is a 'spin' or 'pivot' turn (one wheel forward, one wheel backward).
 */
void turnLeft() {
    // Motor A (Left) Backward
    digitalWrite(MOTOR_A_DIR_PIN, LOW); 
    analogWrite(MOTOR_A_SPEED_PIN, TURN_SPEED);
    
    // Motor B (Right) Forward
    digitalWrite(MOTOR_B_DIR_PIN, HIGH);
    analogWrite(MOTOR_B_SPEED_PIN, TURN_SPEED);
}

/**
 * Turns the robot right.
 * This is a 'spin' or 'pivot' turn (one wheel backward, one wheel forward).
 */
void turnRight() {
    // Motor A (Left) Forward
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);
    analogWrite(MOTOR_A_SPEED_PIN, TURN_SPEED);
    
    // Motor B (Right) Backward
    digitalWrite(MOTOR_B_DIR_PIN, LOW); 
    analogWrite(MOTOR_B_SPEED_PIN, TURN_SPEED);
}

/**
 * Stops all motor movement by setting PWM speed to zero.
 */
void stopMotors() {
    analogWrite(MOTOR_A_SPEED_PIN, 0);
    analogWrite(MOTOR_B_SPEED_PIN, 0);
}
