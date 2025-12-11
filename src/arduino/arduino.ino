/*|----------------------------------------------------------|*/
/*|Smart car movements and serial connections                |*/
/*|if (Serial.available()) doesnt read anything              |*/
/*|----------------------------------------------------------|*/

#include <FastLED.h>

// Ultrasonido
#define TRIG_PIN 13
#define ECHO_PIN 12

// Sensores infrarrojos
#define LEFT_SENSOR_PIN A2
#define CENTER_SENSOR_PIN A1
#define RIGHT_SENSOR_PIN A0

// Led RGB
#define PIN_RBGLED 4 // Neopixel, no sé si led 2 era para otra cosa
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
const int baseSpeed = 90;         // Velocidad base
const int lineThresholdMin = 600; // Deteccion linea franja media-baja
const int lineThresholdMax = 900; // Deteccion linea franja media-alta
const int stopDistanceCm = 8;     // Deteccion obstaculo a 8cm
const int loopDelayMs = 100;      // Delay entre lecturas y acciones

const float kp = 400;            // Ganancia proporcional (ajustar según pruebas)
const float kd = 150;             // Ganancia derivativa (ajustar según pruebas)

// Variables de sensores y control
int leftSensor, centerSensor, rightSensor;
bool lineDetected = false;
bool obstacleDetected = false;
bool canStart = false;
long obstacleDistance;

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

    // Asignacion de valores a sensores segun posicion
    leftSensor = analogRead(LEFT_SENSOR_PIN);
    centerSensor = analogRead(CENTER_SENSOR_PIN);
    rightSensor = analogRead(RIGHT_SENSOR_PIN);
    
    // Deteccion de linea en cualquier sensor
    bool previousLineState = lineDetected;
    lineDetected = (leftSensor > lineThresholdMax) || 
                   (centerSensor > lineThresholdMax) || 
                   (rightSensor > lineThresholdMax);
    
    if (lineDetected) {
        FastLED.showColor(Color(0, 255, 0)); // Verde si linea detectada
    } else {
        FastLED.showColor(Color(255, 0, 0)); // Rojo si no hay linea
    }
    
    if (previousLineState && !lineDetected) {
        Serial.println("LINE_LOST"); // Notificar perdida de linea
    } else if (!previousLineState && lineDetected) {
        Serial.println("LINE_FOUND"); // Notificar linea encontrada
    }
}

// Seguir linea con controlador PD
void followLine() {

  // Negativo = linea a la izquierda, Positivo = linea a la derecha
  if (centerSensor > lineThresholdMax) {
    error = 0;  // Linea centrada
  }
  else if (leftSensor > lineThresholdMax) {
    error = -2;  // Linea muy a la izquierda
  }
  else if (rightSensor > lineThresholdMax) {
    error = 2;   // Linea muy a la derecha
  }
  else if (leftSensor > lineThresholdMin && centerSensor > lineThresholdMin) {
    error = -1;  // Linea ligeramente a la izquierda
  }
  else if (rightSensor > lineThresholdMin && centerSensor > lineThresholdMin) {
    error = 1;   // Linea ligeramente a la derecha
  }
  
  // Calcular derivada (que tan rapido cambia el error)
  derivative = error - lastError;
  
  // Calcular correcion usando PD
  correction = (kp * error) + (kd * derivative);
  
  // Aplicar corrección a los motores
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;
  
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

// Detener motores
void stopMotors() {
    analogWrite(MOTOR_A_SPEED_PIN, 0);
    analogWrite(MOTOR_B_SPEED_PIN, 0);
}

// Calculo distancia ultrasonido preciso
long getDistance() {
    // Limpiar pin trigger
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    
    // Enviar pulso HIGH de 10 microsegundos al pin trigger
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Leer pin echo, pulseIn devuelva la duracion del viaje de la onda
    // Establecer un timeout de 50ms
    long duration = pulseIn(ECHO_PIN, HIGH, 50000); 
    
    // Calculo de la distancia
    // Distancia = (Tiempo * Velocidad del sonido (0.034) / 2
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
    
    // Detener motores al iniciar
    stopMotors();

    // Comprobacion de comunicacion serial con ESP32
    // Solo se hara una vez al inicio
    String sendBuff;

    while(1) {

        if (Serial.available()) {

            char c = Serial.read();
            sendBuff += c;
            
            if (c == '}')  {
                
                // Prints para debugear
                Serial.print("Received data in serial port from ESP32: ");
                Serial.println(sendBuff);
                // ------

                canStart = true;
                Serial.println("START_LAP_READY");
                sendBuff = "";
                break;
            } 
        }
    }
}

void loop() {

    // Inicio solo si se ha establecido comunicacion serial
    if (canStart) {
        // Leer distancia a obstaculos
        obstacleDistance = getDistance();

        // Prints para debugear
        Serial.print("Distance: ");
        Serial.println(obstacleDistance);
        // -----

        // Lectura de sensores infrarrojos
        readSensors();
        
        // Mientras no haya obstaculo cercano, seguir linea
        if (obstacleDistance > stopDistanceCm || obstacleDistance == 0) {
            followLine();
            
        } else {
            // En cualquier otro caso, detenerse
            stopMotors();
        }
    }
    
    // Espera para el siguiente ciclo
    delay(loopDelayMs);
}
