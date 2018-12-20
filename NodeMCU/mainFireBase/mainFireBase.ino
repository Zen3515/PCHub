#include <ArduinoJson.h>
#include <FirebaseArduino.h>
#include <ESP8266WiFi.h>

#define WIFI_SSID "Zen3515 HP Mi8"
#define WIFI_PASSWORD "yvar3515"
#define FIREBASE_HOST "stm32t1.firebaseio.com"
#define FIREBASE_AUTH "pF9mTBahfOEs3u8j330GwYieOgIezDsm5PJMtL5J"

unsigned long prevTime = 0;

void setup() {
  Serial.begin(9600);
  
  // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());

//  Firebase.begin("publicdata-cryptocurrency.firebaseio.com");
//  Firebase.stream("/bitcoin/last");
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
//  Firebase.stream("/");
}


void loop() {
  unsigned long thisTime = millis();
  if (Firebase.failed()) {
    Serial.println("streaming error");
    Serial.println(Firebase.error());
  }
  if(Serial.available() >= 8){
    String temp = Serial.readStringUntil('\n');
    if(thisTime - prevTime >= 1000){
      float temperature = temp.toFloat();
      Firebase.setFloat("temperature", temperature);
    }
//    delay(100);
//    if (Firebase.failed()) {
//      Serial.print("setting /number failed:");
//      Serial.println(Firebase.error());  
//      return;
//    }
  }
  delay(100);

  
//  if (Firebase.available()) {
//     FirebaseObject event = Firebase.readEvent();
//     String eventType = event.getString("type");
//     eventType.toLowerCase();
//     
//     Serial.print("event: ");
//     Serial.println(eventType);
//     if (eventType == "put") {
//       Serial.print("data: ");
//       Serial.println(event.getString("data"));
//       String path = event.getString("path");
//       String data = event.getString("data");
//
//       display.clearDisplay();
//       display.setTextSize(2);
//       display.setTextColor(WHITE);
//       display.setCursor(0,0);
//       display.println(path.c_str()+1);
//       display.println(data);
//       display.display();
//     }
//  }
}
