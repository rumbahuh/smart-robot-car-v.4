/*|----------------------------------------------------------|*/
/*|Connection sketch to local wifi with generic names        |*/
/*|----------------------------------------------------------|*/

#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ArduinoJson.h>

#define EAP_ANONYMOUS_IDENTITY "20220719anonymous@urjc.es"
#define EAP_IDENTITY "mj.mercado.2019@alumnos.urjc.es"
#define EAP_PASSWORD "Bubulubu19@"
#define EAP_USERNAME "mj.mercado.2019@alumnos.urjc.es"
#define WIFI_RETRY_DELAY_MS 500

#define LOCAL_SSID "DCFA"
#define LOCAL_PASSWORD "7pd35r5njnt7xj"

#define MQTT_SERVER "193.147.79.118"
#define MQTT_PORT 21883
#define MQTT_RETRY_DELAY_MS 5000

#define TEAM_ID "16"
#define TEAM_NAME "DIECISEIS"

#define SERIAL_BAUD_RATE 9600
#define SERIAL2_RX_PIN 33
#define SERIAL2_TX_PIN 4

#define PING_INTERVAL_MS 4000
#define LOOP_DELAY_MS 100

String mqtt_topic = "/SETR/2025/" + String(TEAM_ID) + "/";

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT);
Adafruit_MQTT_Publish publisher = Adafruit_MQTT_Publish(&mqtt, mqtt_topic.c_str());

const char* ssid = "eduroam";
unsigned long start_time = 0;
unsigned long last_ping_time = 0;
bool lap_started = false;
bool wifi_connected = false;
bool mqtt_connected = false;

void setup() 
{
    Serial.begin(SERIAL_BAUD_RATE);
    Serial2.begin(SERIAL_BAUD_RATE, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
    
    delay(2000);
    
    connect_to_wifi();
    connect_to_mqtt();
    
    if (wifi_connected && mqtt_connected) {
        Serial2.print("{READY}");
        Serial.println("Ready signal sent to Arduino");
    }
}

void loop()
{
    check_arduino_messages();
    
    if (lap_started) {
        unsigned long current_time = millis();
        if (current_time - last_ping_time >= PING_INTERVAL_MS) {
            send_ping_message();
            last_ping_time = current_time;
        }
    }
    
    delay(LOOP_DELAY_MS);
}

void connect_to_wifi() 
{
    Serial.println("Connecting to WiFi eduroam...");
    WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD); 

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(WIFI_RETRY_DELAY_MS);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        Serial.println("\nWiFi connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi connection failed!");
    }
}

void connect_to_local() {
    Serial.println("Connecting to local WiFi...");
    WiFi.begin(LOCAL_SSID, LOCAL_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(WIFI_RETRY_DELAY_MS);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        Serial.println("\nLocal WiFi connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nLocal WiFi connection failed!");
    }
}

void connect_to_mqtt() 
{
    Serial.println("Connecting to MQTT...");
   
    int8_t ret;
    int retries = 3;
    
    while ((ret = mqtt.connect()) != 0 && retries > 0) {
        Serial.print("MQTT Error: ");
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println("Retrying in 5 seconds...");
        mqtt.disconnect();
        delay(MQTT_RETRY_DELAY_MS);
        retries--;
    }
    
    if (ret == 0) {
        mqtt_connected = true;
        Serial.println("MQTT Connected!");
    } else {
        Serial.println("MQTT connection failed!");
    }
}

void check_arduino_messages() 
{
    if (Serial2.available()) {
        String message = Serial2.readStringUntil('\n');
        message.trim();
        
        Serial.print("Received: ");
        Serial.println(message);
        
        if (message == "START_LAP_READY") {
            send_start_lap_message();
        } else if (message.startsWith("OBSTACLE_DETECTED:")) {
            int distance = message.substring(18).toInt();
            send_obstacle_detected_message(distance);
            send_end_lap_message();
        } else if (message == "LINE_LOST") {
            send_line_lost_message();
        } else if (message == "LINE_FOUND") {
            send_line_found_message();
        }
    }
}

void send_start_lap_message() 
{
    if (!lap_started) {
        DynamicJsonDocument doc(1024);
        doc["team_name"] = TEAM_NAME;
        doc["id"] = TEAM_ID;
        doc["action"] = "START_LAP";
        
        String json_string;
        serializeJson(doc, json_string);

        if (publisher.publish(json_string.c_str())) {
            lap_started = true;
            start_time = millis();
            last_ping_time = millis();
            Serial.println("START_LAP sent successfully");
        } else {
            Serial.println("Failed to send START_LAP");
        }
    }
}

void send_end_lap_message() 
{
    if (lap_started) {
        unsigned long lap_time = millis() - start_time;
        
        DynamicJsonDocument doc(1024);
        doc["team_name"] = TEAM_NAME;
        doc["id"] = TEAM_ID;
        doc["action"] = "END_LAP";
        doc["time"] = lap_time;
        
        String json_string;
        serializeJson(doc, json_string);

        if (publisher.publish(json_string.c_str())) {
            lap_started = false;
            Serial.print("END_LAP sent. Time: ");
            Serial.println(lap_time);
        } else {
            Serial.println("Failed to send END_LAP");
        }
    }
}

void send_obstacle_detected_message(int distance) 
{
    DynamicJsonDocument doc(1024);
    doc["team_name"] = TEAM_NAME;
    doc["id"] = TEAM_ID;
    doc["action"] = "OBSTACLE_DETECTED";
    doc["distance"] = distance;
    
    String json_string;
    serializeJson(doc, json_string);
    
    if (publisher.publish(json_string.c_str())) {
        Serial.print("OBSTACLE_DETECTED sent. Distance: ");
        Serial.println(distance);
    } else {
        Serial.println("Failed to send OBSTACLE_DETECTED");
    }
}

void send_line_lost_message() 
{
    DynamicJsonDocument doc(1024);
    doc["team_name"] = TEAM_NAME;
    doc["id"] = TEAM_ID;
    doc["action"] = "LINE_LOST";
    
    String json_string;
    serializeJson(doc, json_string);
    
    if (publisher.publish(json_string.c_str())) {
        Serial.println("LINE_LOST sent");
    } else {
        Serial.println("Failed to send LINE_LOST");
    }
}

void send_line_found_message() 
{
    DynamicJsonDocument doc(1024);
    doc["team_name"] = TEAM_NAME;
    doc["id"] = TEAM_ID;
    doc["action"] = "LINE_FOUND";
    
    String json_string;
    serializeJson(doc, json_string);
    
    if (publisher.publish(json_string.c_str())) {
        Serial.println("LINE_FOUND sent");
    } else {
        Serial.println("Failed to send LINE_FOUND");
    }
}

void send_ping_message() 
{
    if (lap_started) {
        unsigned long current_time = millis() - start_time;
        
        DynamicJsonDocument doc(1024);
        doc["team_name"] = TEAM_NAME;
        doc["id"] = TEAM_ID;
        doc["action"] = "PING";
        doc["time"] = current_time;
        
        String json_string;
        serializeJson(doc, json_string);
        
        if (publisher.publish(json_string.c_str())) {
            Serial.print("PING sent. Time: ");
            Serial.println(current_time);
        } else {
            Serial.println("Failed to send PING");
        }
    }
}