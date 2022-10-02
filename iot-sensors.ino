#include "DHT.h"
#include <MQUnifiedsensor.h>
#include <WiFi.h>
#include <WiFiClient.h>
//#include <PubSubClient.h>
//
//
//WiFiClient ESPWiFiClient;
//PubSubClient mqttClient(ESPWiFiClient);

#define DHTPIN 4     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
#define placa "ESP-32"
#define Voltage_Resolution 3.3
#define pin 34
#define type "MQ-135"
#define ADC_Bit_Resolution 12
#define RatioMQ135CleanAir 3.6  
// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

double CO2 = (0);  
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
//
//const char* mqtt_broker = "broker.hivemq.com";
//const int mqtt_port = 1883;
//int mqtt_timeout = 10000;
//
//
const char* wifi_ssid = "brisa-1326640";
const char* wifi_password = "t8ikbrnu";
int wifi_timeout = 100000;

//void connectMQTT() {
//  Serial.print("Reconectando ao MQTT Broker..");
//
//  unsigned long startTime = millis();
//  while (!mqttClient.connected() && (millis() - startTime < mqtt_timeout)) {
//      Serial.print(".");
//      String clientId = "client";
//      clientId += String(random(0xffff), HEX);
//     
//      if (mqttClient.connect(clientId.c_str())) {
//        Serial.println();
//        Serial.print("Conectado ao broker MQTT!");
//      }
//      delay(100);
//  }
//  Serial.println();
//}


void connectWiFi(){
  WiFi.mode(WIFI_STA); //"station mode": permite o ESP32 ser um cliente da rede WiFi
  WiFi.begin(wifi_ssid, wifi_password);
  Serial.print("Conectando à rede WiFi .. ");

  unsigned long startTime = millis();
  while(WiFi.status() != WL_CONNECTED && (millis() - startTime < wifi_timeout)){
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  if(WiFi.status() != WL_CONNECTED){
    Serial.println("Falhou!");
  } else{
    Serial.print("Conectado com o IP: ");
    Serial.println(WiFi.localIP());
  }
}



void setup() {
  Serial.begin(9600);
  Serial.println(F("DHTxx test!"));
  connectWiFi();
  if(WiFi.status() == WL_CONNECTED){
//    Serial.println("Conectando ao broker MQTT ...");
//    mqttClient.setServer(mqtt_broker, mqtt_port);
  }



//Set math model to calculate the PPM concentration and the value of constants  
    MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b  
    MQ135.setA(110.47);
    MQ135.setB(-2.862);
    // Configurate the ecuation values to get NH4 concentration    
    MQ135.init();    
    Serial.print("Calibrating please wait.");  
    float calcR0 = 0;  
    for(int i = 1; i<=10; i ++)   {    
        MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin    
        calcR0 += MQ135.calibrate(RatioMQ135CleanAir);    
        Serial.print(".");  
    }  
    MQ135.setR0(calcR0/10);  
    Serial.println("  done!.");      
    if(isinf(calcR0)) { Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply"); while(1);}  
    if(calcR0 == 0){Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply"); while(1);}  
    /*****************************  MQ CAlibration **************************/                  
    MQ135.serialDebug(false);
  dht.begin();
}

void loop() {
  // Wait a few seconds between measurements.
  delay(2000);
//  if (!mqttClient.connected()){
//    connectMQTT();
//  }
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
//
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

//  Serial.print(F("Humidity: "));
//  Serial.print(h);
//  Serial.print(F("%  Temperature: "));
//  Serial.print(t);
//  Serial.print(F("°C "));
//  Serial.print(f);
//  Serial.print(F("°F  Heat index: "));
//  Serial.print(hic);
//  Serial.print(F("°C "));
//  Serial.print(hif);
//  Serial.println(F("°F"));


    MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin  
    CO2 = MQ135.readSensor(); // Sensor will read CO2 concentration using the model and a and b values setted before or in the setup  
    Serial.print("CO2: ");  
    Serial.println(CO2);

   
//  if (mqttClient.connected()){
//    mqttClient.loop();
//   
//    mqttClient.publish("/imd0902/t01/g3",String(h).c_str(), true);
//    delay(3000);
//    mqttClient.publish("/imd0902/t01/g3", String(t).c_str(), true);
//    delay(3000);
//  }
}
