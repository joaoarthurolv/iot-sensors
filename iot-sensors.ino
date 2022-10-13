#include "DHT.h"
#include <MQUnifiedsensor.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>

WiFiClient ESPWiFiClient;
PubSubClient mqtt_client(ESPWiFiClient);

#define DHTPIN 4
#define DHTTYPE DHT11   // DHT 11

#define placa "ESP-32"
#define Voltage_Resolution 3.3
#define pin 34
#define type "MQ-135"
#define ADC_Bit_Resolution 12
#define RatioMQ135CleanAir 3.6


#define IO_USERNAME  "joaoarthurolv"
#define IO_KEY       "aio_YmTT98lT4Sv0I7aVBWGn4qCto5iO"

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

double CO2 = (0);
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

const char* mqtt_broker = "io.adafruit.com";
const int mqtt_port = 1883;
int mqtt_timeout = 10000;

const char* wifi_ssid = "Redmi Note 8";
const char* wifi_password = "danilo123";
int wifi_timeout = 100000;

void connectMQTT() {
  unsigned long tempoInicial = millis();
  while (!mqtt_client.connected() && (millis() - tempoInicial < mqtt_timeout)) {
    if (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
    }
    Serial.print("Conectando ao MQTT Broker..");

    if (mqtt_client.connect("ESP32Client", IO_USERNAME, IO_KEY)) {
      Serial.println();
      Serial.print("Conectado ao broker MQTT!");
    } else {
      Serial.println();
      Serial.print("Conexão com o broker MQTT falhou!");
      delay(500);
    }
  }
  Serial.println();
}

void connectWiFi() {
  WiFi.mode(WIFI_STA); //"station mode": permite o ESP32 ser um cliente da rede WiFi
  WiFi.begin(wifi_ssid, wifi_password);
  Serial.print("Conectando à rede WiFi .. ");

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startTime < wifi_timeout)) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Falhou!");
  } else {
    Serial.print("Conectado com o IP: ");
    Serial.println(WiFi.localIP());
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("DHTxx test!"));

  connectWiFi();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Conectando ao broker MQTT ...");
    mqtt_client.setServer(mqtt_broker, mqtt_port);
  }

  //Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.setA(110.47);
  MQ135.setB(-2.862);
  // Configurate the ecuation values to get NH4 concentration
  MQ135.init();
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i ++)   {
    MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println("  done!.");
  if (isinf(calcR0)) {
    Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply");
    while (1);
  }
  /*****************************  MQ CAlibration **************************/
  MQ135.serialDebug(false);

  dht.begin();
}

void loop() {
  // Wait a few seconds between measurements.
  delay(2000);
    if (!mqtt_client.connected()) {
      connectMQTT();
    }
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  Serial.print(F("Umidade: "));
  Serial.print(h);
  Serial.print(F("%  Temperatura: "));
  Serial.print(t);
  Serial.println(F("°C "));

  MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin
  CO2 = MQ135.readSensor(); // Sensor will read CO2 concentration using the model and a and b values setted before or in the setup
  Serial.print("CO2: ");
  Serial.println(CO2);

  if (mqtt_client.connected()) {
    mqtt_client.publish("joaoarthurolv/feeds/co2", String(CO2).c_str());
    Serial.println("Publicou o dado: " + String(CO2));
    mqtt_client.publish("joaoarthurolv/feeds/temperatura", String(t).c_str());
    Serial.println("Publicou o dado: " + String(t));
    mqtt_client.publish("joaoarthurolv/feeds/umidade", String(h).c_str());
    Serial.println("Publicou o dado: " + String(h));
    delay(10000);
    mqtt_client.loop();
  }
}