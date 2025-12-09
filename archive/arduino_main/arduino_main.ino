#include <FastLED.h>

#define TRIG_PIN 13
#define ECHO_PIN 12
#define LEFT_SENSOR_PIN A2
#define CENTER_SENSOR_PIN A1
#define RIGHT_SENSOR_PIN A0

#define MOTOR_STBY_PIN 3
#define MOTOR_A_DIR_PIN 7
#define MOTOR_A_SPEED_PIN 5
#define MOTOR_B_DIR_PIN 8
#define MOTOR_B_SPEED_PIN 6

#define LED_PIN 2
#define NUM_LEDS 1
#define LED_BRIGHTNESS 50

CRGB leds[NUM_LEDS];

int leftSensor, centerSensor, rightSensor;
long obstacleDistance;
bool lineDetected = false;
bool obstacleDetected = false;
bool canStart = false;

const int BASE_SPEED = 90;
const int TURN_SPEED = 80;
const int LINE_THRESHOLD = 500;
const int MAX_OBSTACLE_DISTANCE = 30;

void setup() {
    Serial.begin(9600);
    
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);
    
    pinMode(MOTOR_STBY_PIN, OUTPUT);
    pinMode(MOTOR_A_DIR_PIN, OUTPUT);
    pinMode(MOTOR_A_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_B_DIR_PIN, OUTPUT);
    pinMode(MOTOR_B_SPEED_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    digitalWrite(MOTOR_STBY_PIN, HIGH);
    setLedColor(255, 0, 0);
    stopMotors();
    
    Serial.println("Arduino is ready. Waiting for confirmation from ESP32...");
}

void loop() {
    checkSerialCommunication();
    
    if (canStart) {
        readSensors();
        checkObstacle();
        
        if (!obstacleDetected) {
            followLine();
        } else {
            stopMotors();
            Serial.println("OBSTACLE_DETECTED:" + String(obstacleDistance));
        }
    }
    
    delay(50);
}

void checkSerialCommunication() {
    if (Serial.available()) {
        String message = Serial.readString();
        message.trim();
        
        if (message == "START_CONFIRMED") {
            canStart = true;
            Serial.println("START_LAP_READY");
        } else if (message == "STOP") {
            canStart = false;
            stopMotors();
        }
    }
}

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

void checkObstacle() {
    obstacleDistance = getDistance();
    
    if (obstacleDistance > 0 && obstacleDistance <= MAX_OBSTACLE_DISTANCE) {
        if (!obstacleDetected) {
            obstacleDetected = true;
            Serial.println("OBSTACLE_DETECTED:" + String(obstacleDistance));
        }
        
        if (obstacleDistance <= 8) {
            stopMotors();
        }
    } else {
        obstacleDetected = false;
    }
}

long getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH);
    long distance = duration * 0.034 / 2;
    
    return distance;
}

void followLine() {
    if (centerSensor > LINE_THRESHOLD) {
        moveForward(BASE_SPEED);
    } else if (leftSensor > LINE_THRESHOLD) {
        turnLeft();
    } else if (rightSensor > LINE_THRESHOLD) {
        turnRight();
    } else {
        stopMotors();
        Serial.println("LINE_SEARCH_NEEDED");
    }
}

void moveForward(int speed) {
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);
    digitalWrite(MOTOR_B_DIR_PIN, HIGH);
    analogWrite(MOTOR_A_SPEED_PIN, speed);
    analogWrite(MOTOR_B_SPEED_PIN, speed);
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

void stopMotors() {
    analogWrite(MOTOR_A_SPEED_PIN, 0);
    analogWrite(MOTOR_B_SPEED_PIN, 0);
}

void setLedColor(int r, int g, int b) {
    leds[0] = CRGB(r, g, b);
    FastLED.show();
}
