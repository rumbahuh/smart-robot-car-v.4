#define LEFT_SENSOR_PIN A2
#define CENTER_SENSOR_PIN A1
#define RIGHT_SENSOR_PIN A0

const int LINE_THRESHOLD = 600;     // 600 minimo
int leftSensor, centerSensor, rightSensor;

void lecturaSensores()
{
  leftSensor = analogRead(LEFT_SENSOR_PIN);
  centerSensor = analogRead(CENTER_SENSOR_PIN);
  rightSensor = analogRead(RIGHT_SENSOR_PIN);
  
  // Print all three values on the same line
  Serial.print("left: ");
  Serial.print(leftSensor);
  Serial.print(" | center: ");
  Serial.print(centerSensor);
  Serial.print(" | right: ");
  Serial.println(rightSensor);
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  lecturaSensores();
  delay(100);
}