/*|----------------------------------------------------------|*/
/*|Smart car movements and serial connections                |*/
/*|if (Serial.available()) doesnt read anything              |*/
/*|----------------------------------------------------------|*/

#include <FastLED.h>

// Ultrasonic Sensor Pins
#define TRIG_PIN 13
#define ECHO_PIN 12

#define LEFT_SENSOR_PIN A2
#define CENTER_SENSOR_PIN A1
#define RIGHT_SENSOR_PIN A0

#define PIN_RBGLED 4
#define LED_PIN 2
#define NUM_LEDS 1
#define LED_BRIGHTNESS 50

// Motor Driver Pins (assuming an L298N-type driver)
#define MOTOR_STBY_PIN 3
#define MOTOR_A_DIR_PIN 7
#define MOTOR_A_SPEED_PIN 5 // PWM for speed
#define MOTOR_B_DIR_PIN 8
#define MOTOR_B_SPEED_PIN 6 // PWM for speed

#define LED_PIN 2
#define NUM_LEDS 1
#define LED_BRIGHTNESS 20

CRGB leds[NUM_LEDS];

// --- CONSTANTS ---
const int BASE_SPEED = 90;
const int TURN_SPEED = 80;
const int LINE_THRESHOLD = 500;
const int STOP_DISTANCE_CM = 8;
const int LOOP_DELAY_MS = 100;       // Delay between sensor readings/actions

int leftSensor, centerSensor, rightSensor;
bool lineDetected = false;
bool obstacleDetected = false;
bool canStart = false;

// --- GLOBAL VARIABLE ---
long obstacleDistance;

 void readSensors() {
    leftSensor = analogRead(LEFT_SENSOR_PIN);
    centerSensor = analogRead(CENTER_SENSOR_PIN);
    rightSensor = analogRead(RIGHT_SENSOR_PIN);
    
    bool previousLineState = lineDetected;
    lineDetected = (leftSensor > LINE_THRESHOLD) || 
                   (centerSensor > LINE_THRESHOLD) || 
                   (rightSensor > LINE_THRESHOLD);
    
    if (lineDetected) {
        setLedColor(0, 255, 0);
    } else {
        setLedColor(255, 0, 0);
    }
    
    if (previousLineState && !lineDetected) {
        Serial.println("LINE_LOST");
    } else if (!previousLineState && lineDetected) {
        Serial.println("LINE_FOUND");
    }
}

void followLine() {
    if (centerSensor > LINE_THRESHOLD) {
        moveForward(BASE_SPEED);
    } else if (leftSensor > LINE_THRESHOLD) {
        turnLeft();
    } else if (rightSensor > LINE_THRESHOLD) {
        turnRight();
    } else {
        int initialTime = millis();
        lineLost(initialTime);
        Serial.println("LINE_SEARCH_NEEDED");
    }
}

void lineLost(int initialTime) {
    turnRight();
    if (rightSensor > LINE_THRESHOLD) {
        if(millis() - initialTime >= 500) {
            turnLeft();
        } else {
            turnRight();
        }
    } 
}

void turnLeft() {
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);
    digitalWrite(MOTOR_B_DIR_PIN, LOW);
    analogWrite(MOTOR_A_SPEED_PIN, TURN_SPEED);
    analogWrite(MOTOR_B_SPEED_PIN, TURN_SPEED);
}

void turnRight() {
    digitalWrite(MOTOR_A_DIR_PIN, LOW);
    digitalWrite(MOTOR_B_DIR_PIN, HIGH);
    analogWrite(MOTOR_A_SPEED_PIN, TURN_SPEED);
    analogWrite(MOTOR_B_SPEED_PIN, TURN_SPEED);
}

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

void setLedColor(int r, int g, int b) {
    leds[0] = CRGB(r, g, b);
    FastLED.show();
}
// --- Ultrasonic Sensor Function ---

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

void setup() {

    Serial.begin(9600);
    Serial.println("--- Motor & Ultrasonic Test Start ---");

    FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);
    
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
    // setLedColor(255, 0, 0);
    FastLED.showColor(Color(255, 0, 0));
    stopMotors();
    Serial.println("System Initialized. Ready to move.");
}

void checkSerialCommunication() {
    String receivedCommand = "";
    if (Serial.available()) {
        String message = Serial.readString();
        receivedCommand = Serial.readStringUntil('\n');
        message.trim();
        receivedCommand.trim();

        if (message == "START_CONFIRMED" || receivedCommand.equals("START_CONFIRMED")) {
            canStart = true;
            Serial.println("START_LAP_READY");
        } else if (message == "STOP") {
            canStart = false;
            stopMotors();
        } else {
            Serial.print("Received unexpected: '");
            Serial.print(receivedCommand);
            Serial.println("'");
        }
    }
}

void loop() {

    checkSerialCommunication();

    if (canStart) {
        // 1. Read the distance
        obstacleDistance = getDistance();

        Serial.print("Distance: ");
        Serial.print(obstacleDistance);
        Serial.print(" cm. Action: ");

        readSensors();
        
        // 2. Implement basic obstacle avoidance logic
        if (obstacleDistance > STOP_DISTANCE_CM || obstacleDistance == 0) {
            followLine();
            
        } else {
            // Obstacle detected close enough to stop
            stopMotors();
            Serial.println("STOPPED (Obstacle too close!)");
        }
    }
    
    // Wait for the defined interval before the next cycle
    delay(LOOP_DELAY_MS);
}
