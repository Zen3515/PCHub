#include <MicroGear.h>
#include <ESP8266WiFi.h>
//#include <SoftwareSerial.h> //we no loger need software serial as serial.write basically send data to uart too.
//#include "DHT.h"

const char* ssid   = "WinZen'WIFI";
const char* password = "bangSUE2054";
int number = 0;

#define APPID   "STM32T1PCHub"
#define KEY  "G9ewlPy9NWkojLl"
#define SECRET  "wqO6Xsq3wk8Tr6bexnmaEg1kM"

#define ALIAS   "NodeMCU1"
#define TargetWeb "HTML_web"

//#define D4 2   // TXD1
//#define DHTPIN D4  // what digital pin we're connected to
//#define DHTTYPE DHT11   // DHT 11

//DHT dht(DHTPIN, DHTTYPE);
//SoftwareSerial uart(D5,D6); //D5 is RX, D6 is TX 

WiFiClient client;
MicroGear microgear(client);

void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) 
{
  Serial.print("Incoming message --> ");
  msg[msglen] = '\0';
  Serial.println((char *)msg);
}


void onConnected(char *attribute, uint8_t* msg, unsigned int msglen) 
{
  Serial.println("Connected to NETPIE...");
  microgear.setAlias(ALIAS);
}

void setup() 
{
   /* Event listener */
  microgear.on(MESSAGE,onMsghandler);
  microgear.on(CONNECTED,onConnected);

//  dht.begin();
  Serial.begin(115200);
  Serial.println("Starting...");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
     delay(250);
     Serial.print(".");
  }

  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  microgear.init(KEY,SECRET,ALIAS);
  microgear.connect(APPID);

//  //inintialize uart
//  pinMode(D5, INPUT);
//  pinMode(D6, OUTPUT);
//  uart.begin(4800);
}

void loop(){
  char uartCommand = 'Z';
  if(uart.available() > 0){
    uartCommand = uart.read();
  }
  switch(uartCommand){
    case 'A':
    break;
    case 'B':
    break;
    default:
    break;
  }
  if (microgear.connected())
  {
     microgear.loop();
     Serial.println("connected");

//     float Humidity = dht.readHumidity();
//     float Temp = dht.readTemperature();  // Read temperature as Celsius (the default)
//     String data = "/" + String(Humidity) + "/" + String(Temp);
//     char msg[128];
//     data.toCharArray(msg,data.length());
//     Serial.println(msg);
     Serial.println("chat to web");
     char msg[128] = "Hello test ";
     msg[10] = number + '0';
     number = (number + 1) % 10;

     microgear.chat(TargetWeb , msg);
  }
   else 
   {
  Serial.println("connection lost, reconnect...");
  microgear.connect(APPID);
   }
  delay(1500);
}
