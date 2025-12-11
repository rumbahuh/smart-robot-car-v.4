/*|----------------------------------------------------------|*/
/*|Smart car movements and serial connections                |*/
/*|----------------------------------------------------------|*/

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

const int baseSpeed = 90;
const int lineThresholdMin = 600;
const int lineThresholdMax = 900;
const int stopDistanceCm = 8;
const int loopDelayMs = 100;

const float kp = 200;
const float kd = 10;

int leftSensor, centerSensor, rightSensor;
bool lineDetected = false;
bool obstacleDetected = false;
bool canStart = false;
bool lapStartSent = false;
long obstacleDistance;

int position = 0;
int lastPosition = 0;
int velocity = 0;
float correction = 0;

const int desiredPosition = 0;

uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}

void readSensors() {
    leftSensor = analogRead(LEFT_SENSOR_PIN);
    centerSensor = analogRead(CENTER_SENSOR_PIN);
    rightSensor = analogRead(RIGHT_SENSOR_PIN);
    
    bool previousLineState = lineDetected;
    lineDetected = (leftSensor > lineThresholdMax) || 
                   (centerSensor > lineThresholdMax) || 
                   (rightSensor > lineThresholdMax);
    
    if (lineDetected) {
        FastLED.showColor(Color(0, 255, 0));
    } else {
        FastLED.showColor(Color(255, 0, 0));
    }
    
    if (previousLineState && !lineDetected) {
        Serial.println("LINE_LOST");
    } else if (!previousLineState && lineDetected) {
        Serial.println("LINE_FOUND");
    }
}

void followLine() {
  if (centerSensor > lineThresholdMax) {
    position = 0;
  }
  else if (leftSensor > lineThresholdMax) {
    position = -2;
  }
  else if (rightSensor > lineThresholdMax) {
    position = 2;
  }
  else if (leftSensor > lineThresholdMin && centerSensor > lineThresholdMin) {
    position = -1;
  }
  else if (rightSensor > lineThresholdMin && centerSensor > lineThresholdMin) {
    position = 1;
  }
  
  velocity = position - lastPosition;
  correction = kp * (desiredPosition - position) - kd * velocity;
  
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;
  
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);
    analogWrite(MOTOR_A_SPEED_PIN, constrain(leftSpeed, 0, 255));
  } else {
    digitalWrite(MOTOR_A_DIR_PIN, LOW);
    analogWrite(MOTOR_A_SPEED_PIN, constrain(-leftSpeed, 0, 255));
  }
  
  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_B_DIR_PIN, HIGH);
    analogWrite(MOTOR_B_SPEED_PIN, constrain(rightSpeed, 0, 255));
  } else {
    digitalWrite(MOTOR_B_DIR_PIN, LOW);
    analogWrite(MOTOR_B_SPEED_PIN, constrain(-rightSpeed, 0, 255));
  }
  
  lastPosition = position;
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

    // Wait for ESP32 ready signal
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
        // Send START_LAP_READY once at the beginning
        if (!lapStartSent) {
            Serial.println("START_LAP_READY");
            lapStartSent = true;
            delay(100);
        }
        
        obstacleDistance = getDistance();
        readSensors();
        
        if (obstacleDistance > stopDistanceCm || obstacleDistance == 0) {
            followLine();
        } else {
            stopMotors();
            Serial.print("OBSTACLE_DETECTED:");
            Serial.println(obstacleDistance);
            while(1); // Stop program
        }
    }
    
    delay(loopDelayMs);
}