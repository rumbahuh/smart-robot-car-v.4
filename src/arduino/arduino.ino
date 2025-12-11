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

const int baseSpeed = 95;
const int lineThresholdMin = 300;
const int lineThresholdMax = 700;
const int stopDistanceCm = 8;
const int loopDelayMs = 50;
const int searchTimeoutMs = 5000;

int leftSensor, centerSensor, rightSensor;
bool lineDetected = false;
bool canStart = false;
bool lapStartSent = false;
bool isSearching = false;
bool searchInitSent = false;
long obstacleDistance;
unsigned long searchStartTime = 0;
int lastKnownDirection = 0;

// Line visibility tracking
unsigned long totalReadings = 0;
unsigned long lineDetectedReadings = 0;

uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}

void readSensors() {
    leftSensor = analogRead(LEFT_SENSOR_PIN);
    centerSensor = analogRead(CENTER_SENSOR_PIN);
    rightSensor = analogRead(RIGHT_SENSOR_PIN);
    
    bool previousLineState = lineDetected;
    lineDetected = (leftSensor > lineThresholdMin) || 
                   (centerSensor > lineThresholdMin) || 
                   (rightSensor > lineThresholdMin);
    
    // Track statistics
    totalReadings++;
    if (lineDetected) {
        lineDetectedReadings++;
        if (leftSensor > lineThresholdMin) lastKnownDirection = -1;
        else if (rightSensor > lineThresholdMin) lastKnownDirection = 1;
        else lastKnownDirection = 0;
    }
    
    if (lineDetected) {
        FastLED.showColor(Color(0, 255, 0));
    } else {
        FastLED.showColor(Color(255, 0, 0));
    }
    
    if (previousLineState && !lineDetected) {
        Serial.println("LINE_LOST");
        isSearching = true;
        searchStartTime = millis();
        searchInitSent = false;
    } else if (!previousLineState && lineDetected) {
        if (isSearching && searchInitSent) {
            Serial.println("STOP_LINE_SEARCH");
        }
        Serial.println("LINE_FOUND");
        isSearching = false;
        searchInitSent = false;
    }
}

void followLine() {
  int leftSpeed, rightSpeed;
  
  if (centerSensor > lineThresholdMax) {
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed;
  }
  else if (leftSensor > lineThresholdMax) {
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed * 0.2;
  }
  else if (rightSensor > lineThresholdMax) {
    leftSpeed = baseSpeed * 0.2;
    rightSpeed = baseSpeed;
  }
  else if (leftSensor > lineThresholdMin && centerSensor > lineThresholdMin) {
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed * 0.5;
  }
  else if (rightSensor > lineThresholdMin && centerSensor > lineThresholdMin) {
    leftSpeed = baseSpeed * 0.5;
    rightSpeed = baseSpeed;
  }
  else if (leftSensor > lineThresholdMin) {
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed * 0.3;
  }
  else if (rightSensor > lineThresholdMin) {
    leftSpeed = baseSpeed * 0.3;
    rightSpeed = baseSpeed;
  }
  else {
    leftSpeed = baseSpeed * 0.6;
    rightSpeed = baseSpeed * 0.6;
  }
  
  digitalWrite(MOTOR_A_DIR_PIN, HIGH);
  digitalWrite(MOTOR_B_DIR_PIN, HIGH);
  analogWrite(MOTOR_A_SPEED_PIN, constrain(leftSpeed, 0, 255));
  analogWrite(MOTOR_B_SPEED_PIN, constrain(rightSpeed, 0, 255));
}

void searchLine() {
    if (!searchInitSent) {
        Serial.println("INIT_LINE_SEARCH");
        searchInitSent = true;
    }
    
    int searchSpeed = baseSpeed * 0.5;
    
    if (lastKnownDirection < 0) {
        digitalWrite(MOTOR_A_DIR_PIN, HIGH);
        digitalWrite(MOTOR_B_DIR_PIN, HIGH);
        analogWrite(MOTOR_A_SPEED_PIN, searchSpeed);
        analogWrite(MOTOR_B_SPEED_PIN, 0);
    } else {
        digitalWrite(MOTOR_A_DIR_PIN, HIGH);
        digitalWrite(MOTOR_B_DIR_PIN, HIGH);
        analogWrite(MOTOR_A_SPEED_PIN, 0);
        analogWrite(MOTOR_B_SPEED_PIN, searchSpeed);
    }
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
    stopMotors();

    String sendBuff;
    while(1) {
        if (Serial.available()) {
            char c = Serial.read();
            sendBuff += c;
            
            if (c == '}')  {
                canStart = true;
                sendBuff = "";
                break;
            } 
        }
    }
    
    delay(500);
}

void loop() {
    if (canStart) {
        if (!lapStartSent) {
            Serial.println("START_LAP_READY");
            lapStartSent = true;
            delay(100);
        }
        
        obstacleDistance = getDistance();
        readSensors();
        
        if (isSearching && (millis() - searchStartTime > searchTimeoutMs)) {
            stopMotors();
            Serial.println("SEARCH_TIMEOUT");
            while(1);
        }
        
        if (obstacleDistance > stopDistanceCm || obstacleDistance == 0) {
            if (isSearching) {
                searchLine();
            } else {
                followLine();
            }
        } else {
            stopMotors();
            
            // Send visibility percentage
            float visibilityPercent = (totalReadings > 0) ? 
                (float)lineDetectedReadings / totalReadings * 100.0 : 0.0;
            Serial.print("VISIBLE_LINE:");
            Serial.println(visibilityPercent);
            delay(100);
            
            // Send obstacle with distance
            Serial.print("OBSTACLE_DETECTED:");
            Serial.println(obstacleDistance);
            while(1);
        }
    }
    
    delay(loopDelayMs);
}