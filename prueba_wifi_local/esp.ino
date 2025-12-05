/*|----------------------------------------------------------|*/
/*|Connection sketch to local wifi with generic names        |*/
/*|As for final push, serial prints should be deleted        |*/
/*|----------------------------------------------------------|*/

#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ArduinoJson.h>

// WiFi configuration
const char* wifi_ssid = "WIFI";
const char* wifi_password = "PASSWORD";

// const char* wifi_ssid = "eduroam";
// const char* wifi_username = "mj.mercado.2019";
// const char* wifi_password = "contraseña_eduroam";

// MQTT configuration
#define MQTT_SERVER "193.147.79.118"
#define MQTT_PORT 21883
#define TEAM_ID "16"
#define TEAM_NAME "DIECISEIS"
#define PING_INTERVAL_MS 4000
#define SERIAL_BAUD_RATE 9600
#define SERIAL2_RX_PIN 16
#define SERIAL2_TX_PIN 17
#define MQTT_RETRY_DELAY_MS 5000
#define LOOP_DELAY_MS 100
#define MAX_LEN 10

// MQTT Topic
String mqtt_topic = "/SETR/2025/" + String(TEAM_ID) + "/";

WiFiClient client;

// MQTT client
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT);
Adafruit_MQTT_Publish publisher = Adafruit_MQTT_Publish(&mqtt, 
                                                        mqtt_topic.c_str());

// Global variables
unsigned long start_time = 0;
unsigned long last_ping_time = 0;
bool lap_started = false;
bool wifi_connected = false;
bool mqtt_connected = false;

void setup() 
{
    Serial.begin(SERIAL_BAUD_RATE);
    Serial2.begin(SERIAL_BAUD_RATE, SERIAL_8N1, SERIAL2_RX_PIN, 
                  SERIAL2_TX_PIN);
    
    Serial.println("ESP32 Starting...");
    
    connect_to_wifi();
    connect_to_mqtt();
    
    if (wifi_connected && mqtt_connected) {
        Serial2.println("START_CONFIRMED");
        Serial.println("System ready. Sent confirmation to Arduino.");
    }
}

void loop() 
{
    if (!mqtt.ping()) {
        mqtt.disconnect();
        connect_to_mqtt();
    }
    
    check_arduino_messages();
    
    if (wifi_connected && mqtt_connected) {
        unsigned long current_time = millis();
        if (current_time - last_ping_time >= PING_INTERVAL_MS) {
            send_heartbeat_message();
            last_ping_time = current_time;
        }
    }

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
    WiFi.begin(wifi_ssid, wifi_password);
    
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    wifi_connected = true;
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void connect_to_mqtt() 
{
    Serial.print("Connecting to MQTT...");
    
    int8_t ret;
    while ((ret = mqtt.connect()) != 0) {
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println("Retrying MQTT connection in 5 seconds...");
        mqtt.disconnect();
        delay(MQTT_RETRY_DELAY_MS);
    }
    
    mqtt_connected = true;
    Serial.println("MQTT Connected!");
}

/*
Función que mandará un mensaje en formato JSON
para llevar cuenta de que las conexión funcionan
cada 4 segundos.
*/
void send_heartbeat_message() 
{
    DynamicJsonDocument doc(256);
    doc["team_name"] = TEAM_NAME;
    doc["id"] = TEAM_ID;
    doc["status"] = "OK";
    doc["timestamp"] = millis();

    String json_string;
    serializeJson(doc, json_string);

    publisher.publish(json_string.c_str());
}

void check_arduino_messages() 
{
    if (Serial2.available()) {
        String message = Serial2.readString();
        message.trim();
        
        Serial.println("Received from Arduino: " + message);
        
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
            Serial.println("START_LAP message sent");
            start_time = millis();
            last_ping_time = millis();
            lap_started = true;
        } else {
            Serial.println("Failed to send START_LAP message");
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
            Serial.println("END_LAP message sent. Time: " + 
                         String(lap_time) + "ms");
            lap_started = false;
        } else {
            Serial.println("Failed to send END_LAP message");
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
        Serial.println("OBSTACLE_DETECTED message sent. Distance: " + 
                     String(distance) + "cm");
    } else {
        Serial.println("Failed to send OBSTACLE_DETECTED message");
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
        Serial.println("LINE_LOST message sent");
    } else {
        Serial.println("Failed to send LINE_LOST message");
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
        Serial.println("LINE_FOUND message sent");
    } else {
        Serial.println("Failed to send LINE_FOUND message");
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
            Serial.println("PING message sent. Time: " + 
                         String(current_time) + "ms");
        } else {
            Serial.println("Failed to send PING message");
        }
    }
}
