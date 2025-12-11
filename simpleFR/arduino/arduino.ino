/*|----------------------------------------------------------|*/
/*|Smart car movements with FreeRTOS                         |*/
/*|----------------------------------------------------------|*/

#include <Arduino_FreeRTOS.h>
#include <FastLED.h>

#define TRIG_PIN 13
#define ECHO_PIN 12
#define LEFT_SENSOR_PIN A2
#define CENTER_SENSOR_PIN A1
#define RIGHT_SENSOR_PIN A0
#define PIN_RBGLED 4
#define NUM_LEDS 1
#define LED_BRIGHTNESS 50
#define MOTOR_STBY_PIN 3
#define MOTOR_A_DIR_PIN 7
#define MOTOR_A_SPEED_PIN 5
#define MOTOR_B_DIR_PIN 8
#define MOTOR_B_SPEED_PIN 6

CRGB leds[NUM_LEDS];

// Constants
const int baseSpeed = 90;
const int turnSpeed = 60;
const int lineThresholdMin = 600;
const int lineThresholdMax = 900;
const int stopDistanceCm = 8;

// Shared variables (volatile for thread safety)
volatile int leftSensor, centerSensor, rightSensor;
volatile bool lineDetected = false;
volatile bool obstacleDetected = false;
volatile bool canStart = false;
volatile bool lapStartSent = false;
volatile long obstacleDistance;
volatile bool previousLineState = false;

// Event flags
volatile bool eventLineLost = false;
volatile bool eventLineFound = false;
volatile bool eventObstacle = false;
volatile int obstacleEventDistance = 0;
volatile bool eventStartLapReady = false;

uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
    return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}

void readSensors() {
    leftSensor = analogRead(LEFT_SENSOR_PIN);
    centerSensor = analogRead(CENTER_SENSOR_PIN);
    rightSensor = analogRead(RIGHT_SENSOR_PIN);
    
    bool currentLineState = (leftSensor > lineThresholdMax) || 
                           (centerSensor > lineThresholdMax) || 
                           (rightSensor > lineThresholdMax);
    
    if (currentLineState) {
        FastLED.showColor(Color(0, 255, 0));
    } else {
        FastLED.showColor(Color(255, 0, 0));
    }
    
    // Detect line state changes
    if (previousLineState && !currentLineState) {
        eventLineLost = true;
    } else if (!previousLineState && currentLineState) {
        eventLineFound = true;
    }
    
    lineDetected = currentLineState;
    previousLineState = currentLineState;
}

void followLine() {
    int left = leftSensor;
    int center = centerSensor;
    int right = rightSensor;
    
    // Center sensor detects line - go straight
    if (center > lineThresholdMax) {
        moveForward(baseSpeed);
    }
    // Left sensor detects line - turn left
    else if (left > lineThresholdMax) {
        turnLeft(turnSpeed);
    }
    // Right sensor detects line - turn right
    else if (right > lineThresholdMax) {
        turnRight(turnSpeed);
    }
    // Multiple sensors detect line - go straight at reduced speed
    else if (center > lineThresholdMin && (left > lineThresholdMin || right > lineThresholdMin)) {
        moveForward(baseSpeed - 20);
    }
    // No clear line detected - stop
    else {
        stopMotors();
    }
}

void moveForward(int speed) {
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);
    analogWrite(MOTOR_A_SPEED_PIN, speed);
    digitalWrite(MOTOR_B_DIR_PIN, HIGH);
    analogWrite(MOTOR_B_SPEED_PIN, speed);
}

void turnLeft(int speed) {
    digitalWrite(MOTOR_A_DIR_PIN, LOW);
    analogWrite(MOTOR_A_SPEED_PIN, speed);
    digitalWrite(MOTOR_B_DIR_PIN, HIGH);
    analogWrite(MOTOR_B_SPEED_PIN, baseSpeed);
}

void turnRight(int speed) {
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);
    analogWrite(MOTOR_A_SPEED_PIN, baseSpeed);
    digitalWrite(MOTOR_B_DIR_PIN, LOW);
    analogWrite(MOTOR_B_SPEED_PIN, speed);
}

void stopMotors() {
    analogWrite(MOTOR_A_SPEED_PIN, 0);
    analogWrite(MOTOR_B_SPEED_PIN, 0);
}

long getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 50000); 
    long distance = duration * 0.034 / 2;
    
    return distance;
}

// Task: Read sensors periodically
void TaskReadSensors(void *pvParameters) {
    (void) pvParameters;
    
    for (;;) {
        if (canStart) {
            readSensors();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task: Read distance sensor
void TaskReadDistance(void *pvParameters) {
    (void) pvParameters;
    
    for (;;) {
        if (canStart) {
            obstacleDistance = getDistance();
            
            if (obstacleDistance > 0 && obstacleDistance <= stopDistanceCm) {
                eventObstacle = true;
                obstacleEventDistance = obstacleDistance;
                obstacleDetected = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Task: Control motors (line following)
void TaskMotorControl(void *pvParameters) {
    (void) pvParameters;
    
    for (;;) {
        if (canStart && !obstacleDetected) {
            if (obstacleDistance > stopDistanceCm || obstacleDistance == 0) {
                followLine();
            } else {
                stopMotors();
            }
        } else if (obstacleDetected) {
            stopMotors();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Task: Handle serial communication and events
void TaskSerialComm(void *pvParameters) {
    (void) pvParameters;
    
    for (;;) {
        // Check for events
        if (eventLineLost) {
            Serial.println("LINE_LOST");
            eventLineLost = false;
        }
        
        if (eventLineFound) {
            Serial.println("LINE_FOUND");
            eventLineFound = false;
        }
        
        if (eventObstacle) {
            Serial.print("OBSTACLE_DETECTED:");
            Serial.println(obstacleEventDistance);
            eventObstacle = false;
        }
        
        if (eventStartLapReady) {
            Serial.println("START_LAP_READY");
            eventStartLapReady = false;
        }
        
        // Send START_LAP_READY once
        if (canStart && !lapStartSent) {
            eventStartLapReady = true;
            lapStartSent = true;
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Task: Wait for ESP32 ready signal
void TaskWaitReady(void *pvParameters) {
    (void) pvParameters;
    
    String sendBuff;
    while(!canStart) {
        if (Serial.available()) {
            char c = Serial.read();
            sendBuff += c;
            
            if (c == '}') {
                canStart = true;
                sendBuff = "";
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Task deletes itself after completion
    vTaskDelete(NULL);
}

void setup() {
    Serial.begin(9600);

    FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);
    
    pinMode(MOTOR_STBY_PIN, OUTPUT);
    pinMode(MOTOR_A_DIR_PIN, OUTPUT);
    pinMode(MOTOR_A_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_B_DIR_PIN, OUTPUT);
    pinMode(MOTOR_B_SPEED_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    digitalWrite(MOTOR_STBY_PIN, HIGH);
    
    // Create tasks
    xTaskCreate(TaskWaitReady, "WaitReady", 128, NULL, 3, NULL);
    xTaskCreate(TaskReadSensors, "ReadSensors", 128, NULL, 2, NULL);
    xTaskCreate(TaskReadDistance, "ReadDistance", 128, NULL, 2, NULL);
    xTaskCreate(TaskMotorControl, "MotorControl", 128, NULL, 2, NULL);
    xTaskCreate(TaskSerialComm, "SerialComm", 256, NULL, 1, NULL);
    
    // Start scheduler
    vTaskStartScheduler();
}

void loop() {
    // Empty - FreeRTOS tasks handle everything
}