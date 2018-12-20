#include <ArduinoJson.h>
#include <FirebaseArduino.h>
#include <ESP8266WiFi.h>
//#include <string.h>

#define WIFI_SSID "Zen3515 HP Mi8"
#define WIFI_PASSWORD "yvar3515"
#define FIREBASE_HOST "stm32t1.firebaseio.com"
#define FIREBASE_AUTH "pF9mTBahfOEs3u8j330GwYieOgIezDsm5PJMtL5J"

#define FAN "/Fan"
#define FANX "data/X"
#define FANY "data/Y"

#define LIGHT "/Light"
#define LIGHTMODE "data/lightMode"
#define COLOR1R "data/color1/R"
#define COLOR1G "data/color1/G"
#define COLOR1B "data/color1/B"
#define COLOR2R "data/color2/R"
#define COLOR2G "data/color2/G"
#define COLOR2B "data/color2/B"

#define LIGHTSPEED "/LightSpeed"

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

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.stream("/embeded");
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
      prevTime = thisTime;
    }
  }

  
  if (Firebase.available()) {
     FirebaseObject event = Firebase.readEvent();
     String eventType = event.getString("type");
     eventType.toLowerCase();
     
//     Serial.print("event: ");
//     Serial.println(eventType);
     
     if (eventType == "put") {
//        Serial.print("data: ");
//        Serial.println(event.getString("data"));
//        Serial.print("path: ");
//        Serial.println(event.getString("path"));
        String path = event.getString("path");
//       String data = event.getString("data");

        if(path == LIGHT){
//          String temp = "LS" + event.getString(LIGHTSPEED);
//          Serial.print(temp);
//          delay(200);
          String temp = "LM" + event.getString(LIGHTMODE);
          Serial.print(temp);
          if(temp != "LM00"){
            String color1 = event.getString(COLOR1R) + event.getString(COLOR1G) + event.getString(COLOR1B);
            delay(100);
            Serial.print(color1);
            String color2 = event.getString(COLOR2R) + event.getString(COLOR2G) + event.getString(COLOR2B);
            delay(100);
            Serial.print(color2);
          }
        } else if(path == FAN){
          Serial.print("FSPD");
          for(int i = 1; i <= 4; i++){
            String n = String(i);
            String pointX = event.getString(FANX + n);
            String pointY = event.getString(FANY + n);
            Serial.print(pointX + pointY);
            delay(200);
          }
        } else if(path == LIGHTSPEED){
          String temp = "LS" + event.getString("data");
          Serial.print(temp);
        }
//
//       display.println(path.c_str()+1);
//       display.println(data);
//        print()
        
     }
  }
  delay(100);
}
