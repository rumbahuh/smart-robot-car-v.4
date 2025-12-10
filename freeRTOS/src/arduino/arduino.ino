/*|----------------------------------------------------------|*/
/*|Smart car movements and serial connections                |*/
/*|----------------------------------------------------------|*/

#include <Arduino_FreeRTOS.h>
#include <FastLED.h>

// Ultrasonido
#define TRIG_PIN 13
#define ECHO_PIN 12

// Sensores infrarrojos
#define LEFT_SENSOR_PIN A2
#define CENTER_SENSOR_PIN A1
#define RIGHT_SENSOR_PIN A0

// Led RGB
#define LED_PIN 2
#define PIN_RBGLED 4
#define NUM_LEDS 1
#define LED_BRIGHTNESS 50

// Motores
#define MOTOR_STBY_PIN 3
#define MOTOR_A_DIR_PIN 7
#define MOTOR_A_SPEED_PIN 5
#define MOTOR_B_DIR_PIN 8
#define MOTOR_B_SPEED_PIN 6

// Variables globales
CRGB leds[NUM_LEDS];

// Constantes
const int baseSpeed = 90;
const int lineThresholdMin = 600;
const int lineThresholdMax = 900;
const int stopDistanceCm = 8;
const int loopDelayMs = 100;

const float kp = 400;
const float kd = 150;

// Variables de sensores y control
volatile int leftSensor, centerSensor, rightSensor;
volatile bool lineDetected = false;
volatile long obstacleDistance = 0;
volatile bool canStart = false;  // acceso compartido desde Serial

// Variables del controlador PD
int error = 0;
int lastError = 0;
int derivative = 0;
int correction = 0;

// Funcion para registrar color dado en formato correcto
uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
    return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}

// Lectura de sensores infrarrojos
void readSensors() {
    leftSensor = analogRead(LEFT_SENSOR_PIN);
    centerSensor = analogRead(CENTER_SENSOR_PIN);
    rightSensor = analogRead(RIGHT_SENSOR_PIN);

    bool previousLineState = lineDetected;
    lineDetected = (leftSensor > lineThresholdMax) || 
                   (centerSensor > lineThresholdMax) || 
                   (rightSensor > lineThresholdMax);

    if (previousLineState && !lineDetected) {
        Serial.println("LINE_LOST");
    } else if (!previousLineState && lineDetected) {
        Serial.println("LINE_FOUND");
    }
}

// Seguir linea con controlador PD
void followLine() {
    if (centerSensor > lineThresholdMax) error = 0;
    else if (leftSensor > lineThresholdMax) error = -2;
    else if (rightSensor > lineThresholdMax) error = 2;
    else if (leftSensor > lineThresholdMin && centerSensor > lineThresholdMin) error = -1;
    else if (rightSensor > lineThresholdMin && centerSensor > lineThresholdMin) error = 1;

    derivative = error - lastError;
    correction = (kp * error) + (kd * derivative);

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

    lastError = error;
}

// Detener motores
void stopMotors() {
    analogWrite(MOTOR_A_SPEED_PIN, 0);
    analogWrite(MOTOR_B_SPEED_PIN, 0);
}

// Calculo distancia ultrasonido
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

/**
 * Task 1: Read sensors
 */
void TaskReadSensors(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        readSensors();
        obstacleDistance = getDistance();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(loopDelayMs));
    }
}

/**
 * Task 2: Follow line
 */
void TaskLineFollower(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        if (canStart && lineDetected && (obstacleDistance > stopDistanceCm || obstacleDistance == 0)) {
            followLine();
        } else {
            stopMotors();
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(loopDelayMs));
    }
}

/**
 * Task 3: LED/status
 */
void TaskLED(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        if (lineDetected) FastLED.showColor(Color(0, 255, 0));
        else FastLED.showColor(Color(255, 0, 0));
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

/**
 * Task 4: Serial reading
 */
void TaskSerial(void* pvParameters) {
    String sendBuff = "";
    while (1) {
        while (Serial.available()) {
            char c = Serial.read();
            sendBuff += c;
            if (c == '}') {
                Serial.print("Received data from ESP32: ");
                Serial.println(sendBuff);
                canStart = true;
                sendBuff = "";
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // No bloqueante
    }
}

/**
 * Task 5: Obstacle handling (preemptive)
 */
void TaskObstacle(void* pvParameters) {
    while (1) {
        if (canStart && obstacleDistance > 0 && obstacleDistance <= stopDistanceCm) {
            stopMotors();  // Detener inmediatamente
            Serial.println("OBSTACLE DETECTED! Motors stopped.");
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Chequeo rápido, no bloqueante
    }
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

    // Crear tareas
    xTaskCreate(TaskReadSensors, "SensorTask", 256, NULL, 2, NULL); // Prioridad alta
    xTaskCreate(TaskLineFollower, "LineTask", 256, NULL, 1, NULL);
    xTaskCreate(TaskLED, "LEDTask", 128, NULL, 1, NULL);
    xTaskCreate(TaskSerial, "SerialTask", 256, NULL, 1, NULL);
    xTaskCreate(TaskObstacle, "ObstacleTask", 256, NULL, 3, NULL); // Prioridad máxima

    vTaskStartScheduler();
}

void loop() {
    // Vacío con FreeRTOS
}
