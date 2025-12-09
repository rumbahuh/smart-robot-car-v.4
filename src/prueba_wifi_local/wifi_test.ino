/*|----------------------------------------------------------|*/
/*|Connection sketch to eduroam network (WPA/WPA2) Enteprise |*/
/*|Suitable for almost any ESP32 microcontroller with WiFi   |*/
/*|Raspberry or Arduino WiFi CAN'T USE THIS LIBRARY!!!       |*/
/*|Edited by: Martin Chlebovec (martinius96)                 |*/
/*|Compilation under 2.0.3 Arduino Core and higher worked    |*/
/*|Compilation can be done only using STABLE releases        |*/
/*|Dev releases WILL NOT WORK. (Check your Ard. Core .json)  |*/
/*|WiFi.begin() have more parameters for PEAP connection     |*/
/*|                                                          |*/
/*|Second edit by: Rebeca Castilla (rumbahuh)                |*/
/*|We were having problems with eduroam thus I checked       |*/
/*|locally. Functioning locally                              |*/
/*|----------------------------------------------------------|*/

#include <WiFi.h>

// Recommendation. The following information is sensitive. Better
// if you save those variables in a different header file and make sure that
// new file is not pushed at any github public repository
//#define EAP_ANONYMOUS_IDENTITY "20220719anonymous@urjc.es"
//#define EAP_IDENTITY "USER@urjc.es"
//#define EAP_PASSWORD "PASSWORD"
//#define EAP_USERNAME "USER@urjc.es"

const char* ssid = "WIFI_NAME";
const char* password = "PASSWORD";

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.print(F("Connecting to network: "));
  Serial.println(ssid);
  WiFi.disconnect(true);

  //WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println("");
  Serial.println(F("WiFi is connected!"));
  Serial.println(F("IP address set: "));
  Serial.println(WiFi.localIP());
}

void loop() {
  yield();
}

