// Combined Test Sketch: Verifies motor movement functions and ultrasonic sensor interaction.

// --- PIN DEFINITIONS ---
// Ultrasonic Sensor Pins
#define TRIG_PIN 13
#define ECHO_PIN 12

// Motor Driver Pins (assuming an L298N-type driver)
#define MOTOR_STBY_PIN 3
#define MOTOR_A_DIR_PIN 7
#define MOTOR_A_SPEED_PIN 5 // PWM for speed
#define MOTOR_B_DIR_PIN 8
#define MOTOR_B_SPEED_PIN 6 // PWM for speed

// --- CONSTANTS ---
const int BASE_SPEED = 150;          // Speed for moving forward (0-255)
const int STOP_DISTANCE_CM = 20;     // Obstacle threshold: stop if distance is <= 20 cm
const int LOOP_DELAY_MS = 100;       // Delay between sensor readings/actions

// --- GLOBAL VARIABLE ---
long obstacleDistance;

void setup() {
    Serial.begin(9600);
    Serial.println("--- Motor & Ultrasonic Test Start ---");
    
    // 1. Configure Motor Driver Pins
    pinMode(MOTOR_STBY_PIN, OUTPUT);
    pinMode(MOTOR_A_DIR_PIN, OUTPUT);
    pinMode(MOTOR_A_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_B_DIR_PIN, OUTPUT);
    pinMode(MOTOR_B_SPEED_PIN, OUTPUT);
    
    // 2. Configure Ultrasonic Sensor Pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    // 3. Enable the Motor Driver and stop motors initially
    digitalWrite(MOTOR_STBY_PIN, HIGH);
    stopMotors();
    Serial.println("System Initialized. Ready to move.");
}

void loop() {
    // 1. Read the distance
    obstacleDistance = getDistance();
    
    Serial.print("Distance: ");
    Serial.print(obstacleDistance);
    Serial.print(" cm. Action: ");

    // 2. Implement basic obstacle avoidance logic
    if (obstacleDistance > STOP_DISTANCE_CM || obstacleDistance == 0) {
        // Distance is clear (or reading failed/out of range, which we treat as clear for safety)
        moveForward(BASE_SPEED);
        Serial.println("MOVING FORWARD");
    } else {
        // Obstacle detected close enough to stop
        stopMotors();
        Serial.println("STOPPED (Obstacle too close!)");
    }
    
    // Wait for the defined interval before the next cycle
    delay(LOOP_DELAY_MS);
}

// --- Motor Control Functions ---

/**
 * Moves the robot forward at the specified speed.
 * @param speed PWM value (0-255).
 */
void moveForward(int speed) {
    // Set both motors to move in the forward direction
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);
    digitalWrite(MOTOR_B_DIR_PIN, HIGH);
    
    // Set the speed using PWM
    analogWrite(MOTOR_A_SPEED_PIN, speed);
    analogWrite(MOTOR_B_SPEED_PIN, speed);
}

/**
 * Stops all motor movement by setting PWM speed to zero.
 */
void stopMotors() {
    analogWrite(MOTOR_A_SPEED_PIN, 0);
    analogWrite(MOTOR_B_SPEED_PIN, 0);
}

// --- Ultrasonic Sensor Function ---

/**
 * Measures the distance using the HC-SR04 sensor in centimeters.
 * @return distance in centimeters, or 0 if timeout occurs (out of range).
 */
long getDistance() {
    // 1. Clear the trigger pin
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    
    // 2. Send a 10 microsecond HIGH pulse to the trigger pin
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // 3. Read the echo pin: pulseIn returns the duration of the sound wave travel
    // Set a timeout of 50ms (50000 microseconds)
    long duration = pulseIn(ECHO_PIN, HIGH, 50000); 
    
    // 4. Calculate the distance
    // Distance = (Time * Speed of Sound (0.034 cm/µs)) / 2
    long distance = duration * 0.034 / 2;
    
    return distance;
}
