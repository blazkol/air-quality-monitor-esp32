#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <MQUnifiedsensor.h>

#define DHT_TYPE DHT11
#define DHT_PIN 27
#define MQ135_TYPE "MQ135"
#define MQ135_PIN 34
#define MQ135_CLEAN_AIR_RATIO 3.6
#define MQ9_TYPE "MQ9"
#define MQ9_PIN 35
#define MQ9_CLEAN_AIR_RATIO 9.6
#define BOARD "ESP-32"
#define VOLTAGE_RESOLUTION 3.3
#define ADC_BIT_RESOLUTION 12

const char *wifi_ssid = "WIFI_SSID";
const char *wifi_password = "WIFI_PASSWORD";

const char *mqtt_server = "192.168.0.150";
const char *mqtt_username = "blazkol";
const char *mqtt_password = "blazkol";

const char *temperature_topic = "esp32/temperature";
const char *humidity_topic = "esp32/humidity";
const char *alcohol_topic = "esp32/alcohol";
const char *co2_topic = "esp32/co2";
const char *toluen_topic = "esp32/toluen";
const char *nh4_topic = "esp32/nh4";
const char *acetone_topic = "esp32/acetone";
const char *co_topic = "esp32/co";
const char *lpg_topic = "esp32/lpg";
const char *methane_topic = "esp32/methane";

WiFiClient espClient;
PubSubClient client(espClient);

DHT dht(DHT_PIN, DHT_TYPE);
MQUnifiedsensor MQ135(BOARD, VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, MQ135_PIN, MQ135_TYPE);
MQUnifiedsensor MQ9(BOARD, VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, MQ9_PIN, MQ9_TYPE);

float temperature, humidity, alcohol, co2, toluen, nh4, acetone, co, lpg, methane;

void setup_wifi();
void mqtt_reconnect();
void setup_mq135();
void setup_mq9();
void dht11_measure();
void mq135_measure();
void mq9_measure();

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  mqtt_reconnect();
  dht.begin();
  setup_mq135();
  setup_mq9();
  delay(1000);
}

void loop() {
  if (!client.connected()) {
    mqtt_reconnect();
  }
  dht11_measure();
  mq135_measure();
  mq9_measure();
  delay(1000);
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqtt_reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("esp32-client", mqtt_username, mqtt_password)) {
      Serial.println("connected to MQTT");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup_mq135() {
  MQ135.setRegressionMethod(1);
  MQ135.init();
  Serial.print("Calibrating MQ135. Please wait.");

  float calcR0 = 0;
  for(int i = 1; i<=10; i ++) {
    MQ135.update();
    calcR0 += MQ135.calibrate(MQ135_CLEAN_AIR_RATIO);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println(" Done!");
  
  if(isinf(calcR0)) {
    Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while(1);
  }
  if(calcR0 == 0) {
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while(1);
  }
}

void setup_mq9() {
  MQ9.setRegressionMethod(1);
  MQ9.init();
  Serial.print("Calibrating MQ9. Please wait.");

  float calcR0 = 0;
  for(int i = 1; i<=10; i ++) {
    MQ9.update();
    calcR0 += MQ9.calibrate(MQ9_CLEAN_AIR_RATIO);
    Serial.print(".");
  }
  MQ9.setR0(calcR0/10);
  Serial.println(" Done!");
  
  if(isinf(calcR0)) {
    Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while(1);
  }
  if(calcR0 == 0) {
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while(1);
  }
}

void dht11_measure() {
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  char payload[10];
  if(!isnan(temperature)) {
    sprintf(payload, "%.2f", temperature);
    Serial.print("Temperature payload: ");
    Serial.println(payload);
    client.publish(temperature_topic, payload);
  }
  if(!isnan(humidity)) {
    sprintf(payload, "%.2f", humidity);
    Serial.print("Humidity payload: ");
    Serial.println(payload);
    client.publish(humidity_topic, payload);
  }
  Serial.println("");
}

void mq135_measure() {
  MQ135.update();

  /*
  Exponential regression:
  GAS      | a      | b
  Alcohol  | 77.255 | -3.18 
  CO2      | 110.47 | -2.862
  Toluen   | 44.947 | -3.445
  NH4      | 102.2  | -2.473
  Acetone   | 34.668 | -3.369
  */

  MQ135.setA(77.255); //Configure the equation to calculate Alcohol concentration value
  MQ135.setB(-3.18);
  alcohol = MQ135.readSensor();

  MQ135.setA(110.47); // Configure the equation to calculate CO2 concentration value
  MQ135.setB(-2.862);
  co2 = MQ135.readSensor();

  MQ135.setA(44.947); // Configure the equation to calculate Toluen concentration value
  MQ135.setB(-3.445);
  toluen = MQ135.readSensor();
  
  MQ135.setA(102.2); // Configure the equation to calculate NH4 concentration value
  MQ135.setB(-2.473);
  nh4 = MQ135.readSensor();

  MQ135.setA(34.668); // Configure the equation to calculate Acetone concentration value
  MQ135.setB(-3.369);
  acetone = MQ135.readSensor();

  char payload[10];
  sprintf(payload, "%.2f", alcohol);
  Serial.print("Alcohol payload: ");
  Serial.println(payload);
  client.publish(alcohol_topic, payload);
  sprintf(payload, "%.2f", co2);
  Serial.print("CO2 payload: ");
  Serial.println(payload);
  client.publish(co2_topic, payload);
  sprintf(payload, "%.2f", toluen);
  Serial.print("Toluen payload: ");
  Serial.println(payload);
  client.publish(toluen_topic, payload);
  sprintf(payload, "%.2f", nh4);
  Serial.print("NH4 payload: ");
  Serial.println(payload);
  client.publish(nh4_topic, payload);
  sprintf(payload, "%.2f", acetone);
  Serial.print("Acetone payload: ");
  Serial.println(payload);
  client.publish(acetone_topic, payload);
  Serial.println("");

  delay(500); // Sampling frequency
}

void mq9_measure() {
  MQ9.update();

  /*
  Exponential regression:
  GAS     | a      | b
  CO      | 599.65 | -2.244
  LPG     | 1000.5 | -2.186
  Methane | 4269.6 | -2.648
  */

  MQ9.setA(599.65); // Configure the equation to calculate CO concentration
  MQ9.setB(-2.244); 
  co = MQ9.readSensor();

  MQ9.setA(1000.5); // Configure the equation to calculate LPG concentration
  MQ9.setB(-2.186); 
  lpg = MQ9.readSensor();

  MQ9.setA(4269.6); // Configure the equation to calculate Methane concentration
  MQ9.setB(-2.648);
  methane = MQ9.readSensor();

  char payload[10];
  sprintf(payload, "%.2f", co);
  Serial.print("CO payload: ");
  Serial.println(payload);
  client.publish(co_topic, payload);
  sprintf(payload, "%.2f", lpg);
  Serial.print("LPG payload: ");
  Serial.println(payload);
  client.publish(lpg_topic, payload);
  sprintf(payload, "%.2f", methane);
  Serial.print("Methane payload: ");
  Serial.println(payload);
  client.publish(methane_topic, payload);
  Serial.println("");

  delay(500); // Sampling frequency
}