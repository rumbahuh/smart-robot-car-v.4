
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
#define NUM_LEDS 1
#define LED_BRIGHTNESS 50

// Motores
#define MOTOR_STBY_PIN 3
#define MOTOR_A_DIR_PIN 7
#define MOTOR_A_SPEED_PIN 5 // PWM for speed
#define MOTOR_B_DIR_PIN 8
#define MOTOR_B_SPEED_PIN 6 // PWM for speed

CRGB leds[NUM_LEDS];

// Constantes
const int BASE_SPEED = 90;
const int TURN_SPEED = 80;
const int LINE_THRESHOLD_MIN = 600; // Deteccion linea franja media-baja
const int LINE_THRESHOLD_MAX = 900; // Deteccion linea franja media-alta
const int STOP_DISTANCE_CM = 8;     // Deteccion obstaculo <= 8cm
const int LOOP_DELAY_MS = 100;      // Delay entre lecturas y acciones

int leftSensor, centerSensor, rightSensor;
bool lineDetected = false;
bool obstacleDetected = false;
bool canStart = false;

long obstacleDistance;
// Constantes del controlador PD
const float Kp = 1500;  // Ganancia proporcional (ajustar según pruebas)
const float Kd = 600;  // Ganancia derivativa (ajustar según pruebas)

// Variables del PD
int error = 0;
int lastError = 0;
int derivative = 0;
int correction = 0;

void readSensors() {
    leftSensor = analogRead(LEFT_SENSOR_PIN);
    centerSensor = analogRead(CENTER_SENSOR_PIN);
    rightSensor = analogRead(RIGHT_SENSOR_PIN);
    
    bool previousLineState = lineDetected;
    lineDetected = (leftSensor > LINE_THRESHOLD_MAX) || 
                   (centerSensor > LINE_THRESHOLD_MAX) || 
                   (rightSensor > LINE_THRESHOLD_MAX);
    
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

    // Print all three values on the same line
    Serial.print("left: ");
    Serial.print(leftSensor);
    Serial.print(" | center: ");
    Serial.print(centerSensor);
    Serial.print(" | right: ");
    Serial.println(rightSensor);
}

void followLine() {
  // Calcular ERROR (posición de la línea)
  // Negativo = línea a la izquierda, Positivo = línea a la derecha
  if (centerSensor > LINE_THRESHOLD_MAX) {
    error = 0;  // Línea centrada
  }
  else if (leftSensor > LINE_THRESHOLD_MAX) {
    error = -2;  // Línea muy a la izquierda
  }
  else if (rightSensor > LINE_THRESHOLD_MAX) {
    error = 2;   // Línea muy a la derecha
  }
  else if (leftSensor > LINE_THRESHOLD_MIN && centerSensor > LINE_THRESHOLD_MIN) {
    error = -1;  // Línea ligeramente a la izquierda
  }
  else if (rightSensor > LINE_THRESHOLD_MIN && centerSensor > LINE_THRESHOLD_MIN) {
    error = 1;   // Línea ligeramente a la derecha
  }
  
  // Calcular DERIVADA (qué tan rápido cambia el error)
  derivative = error - lastError;
  
  // Calcular CORRECCIÓN usando PD
  correction = (Kp * error) + (Kd * derivative);
  
  // Aplicar corrección a los motores
  int leftSpeed = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;
  
  // Controlar dirección según velocidad resultante
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);  // Motor izquierdo adelante
    analogWrite(MOTOR_A_SPEED_PIN, constrain(leftSpeed, 0, 255));
  } else {
    digitalWrite(MOTOR_A_DIR_PIN, LOW);   // Motor izquierdo atrás
    analogWrite(MOTOR_A_SPEED_PIN, constrain(-leftSpeed, 0, 255));
  }
  
  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_B_DIR_PIN, HIGH);  // Motor derecho adelante
    analogWrite(MOTOR_B_SPEED_PIN, constrain(rightSpeed, 0, 255));
  } else {
    digitalWrite(MOTOR_B_DIR_PIN, LOW);   // Motor derecho atrás
    analogWrite(MOTOR_B_SPEED_PIN, constrain(-rightSpeed, 0, 255));
  }
  
  // Guardar error actual para próxima iteración
  lastError = error;
}

void lineLost(int initialTime) {
    // por hacer
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

    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
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
    setLedColor(255, 0, 0);
    stopMotors();
    Serial.println("System Initialized. Ready to move.");
}

void loop() {
    // 1. Read the distance
    obstacleDistance = getDistance();
    
    /*
    Serial.print("Distance: ");
    Serial.print(obstacleDistance);
    Serial.print(" cm. Action: ");
    */

    readSensors();

    // 2. Implement basic obstacle avoidance logic
    if (obstacleDistance > STOP_DISTANCE_CM || obstacleDistance == 0) {
        followLine();
        
    } else {
        // Obstacle detected close enough to stop
        stopMotors();
        Serial.println("STOPPED (Obstacle too close!)");
    }
    
    // Wait for the defined interval before the next cycle
    delay(LOOP_DELAY_MS);
}
