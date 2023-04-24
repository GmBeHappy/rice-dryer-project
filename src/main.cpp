#include <Arduino.h>
// MQTT and WiFi
#include <WiFi.h>
#include <PubSubClient.h>

// Sensor
#include <Adafruit_Sensor.h>
#include <DHT.h>

// TFT
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// define pins for TFT screen
#define TFT_CS 5
#define TFT_RST 4
#define TFT_DC 2
#define TFT_SCLK 18
#define TFT_MOSI 23

// define pins for Voltage Meter
#define VOLTAGE_METER_PIN1 34
#define VOLTAGE_METER_PIN2 35

// define pins for DHT21
#define DHTPIN_1 21
#define DHTPIN_2 22
#define DHTTYPE DHT21

// define pins for Relay
#define FAN_RELAY 25
#define HEAT_RELAY 26

// define delay time
long long lastRefreshTime = 0;
long long refreshDelay = 1000;
long long lastPublishTime = 0;
long long publishDelay = 2000;

// Replace with your network credentials
const char *ssid = "rcd";
const char *password = "rcdrcd@1234";

// const char *ssid = "PUIP";
// const char *password = "0640462517";

// const char *ssid = "gm 2.4G";
// const char *password = "gm969/13";

// Replace with your MQTT broker address
const char *mqttServer = "154.215.14.235";
const int mqttPort = 3007;
const char *mqttUser = "test-rcd";
const char *mqttPassword = "test-rcd";
const char *mqttClientId = "RiceDryerEsp32";

// relay variables
boolean fanRelay = false;
boolean heatRelay = false;
boolean heatStatus = false;

DHT dht1(DHTPIN_1, DHTTYPE);
DHT dht2(DHTPIN_2, DHTTYPE);

// safty variables
float maxTemp = 30.0;
float minTemp = 20.0;

WiFiClient espClient;
PubSubClient client(espClient);

// initialize the TFT screen
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// function prototypes
float calcVoltage(int pin);
void reconnect();
void callback(char *topic, byte *message, unsigned int length);

void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);

  // set pinMode for Relay
  pinMode(FAN_RELAY, OUTPUT);
  pinMode(HEAT_RELAY, OUTPUT);

  // set resolution for analogRead
  analogReadResolution(12);

  // Initialize the TFT screen
  tft.initR(INITR_BLACKTAB); // initialize a ST7735S chip, black tab
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(3);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);

  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    tft.setCursor(30, 45); // (x,y) x แนวนอน , y แนวตั้ง
    tft.printf("Connecting to Wifi");

    // show ssid and password
    tft.setCursor(25, 60);
    tft.printf("SSID: %s", ssid);
    tft.setCursor(25, 75);
    tft.printf("Password: %s", password);
  }
  Serial.println("Connected to WiFi");

  // show connected to wifi and ip address
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(30, 45);
  tft.printf("Connected to Wifi");
  tft.setCursor(10, 60);
  tft.printf("IP Address: %s", WiFi.localIP().toString().c_str());

  delay(2000);

  // Connect to MQTT broker
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  while (!client.connected())
  {
    Serial.println("Connecting to MQTT...");

    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(30, 45);
    tft.printf("Connecting to MQTT");
    if (client.connect(mqttClientId, mqttUser, mqttPassword))
    {
      Serial.println("Connected to MQTT");

      client.subscribe("gm/dev/rice/control/#");

      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(30, 45);
      tft.printf("Connected to MQTT");
    }
    else
    {
      Serial.print("Failed with state ");
      Serial.print(client.state());

      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(30, 45);
      tft.printf("Failed with state");
      tft.setCursor(30, 60);
      tft.printf("%d", client.state());
      delay(2000);
    }
  }

  dht1.begin();
  dht2.begin();

  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(30, 45);
  tft.printf("DHT21 Sensor");
  tft.setCursor(30, 60);
  tft.printf("Ready");

  delay(2000);
}

void loop()
{
  // check if we're connected to WiFi and MQTT broker
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  if (millis() - lastRefreshTime > refreshDelay)
  {
    // read temperature and humidity from DHT21
    float t1 = dht1.readTemperature();
    float h1 = dht1.readHumidity();
    float t2 = dht2.readTemperature();
    float h2 = dht2.readHumidity();

    // display data on TFT screen
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(30, 45);
    tft.printf("Sensor 1");
    tft.setCursor(30, 60);
    tft.printf("Temp: %.1f", t1);
    tft.setCursor(30, 75);
    tft.printf("Hum: %.1f", h1);

    tft.setCursor(30, 90);
    tft.printf("Sensor 2");
    tft.setCursor(30, 105);
    tft.printf("Temp: %.1f", t2);
    tft.setCursor(30, 120);
    tft.printf("Hum: %.1f", h2);

    lastRefreshTime = millis();
  }

  if (millis() - lastPublishTime > publishDelay)
  {
    // read temperature and humidity from DHT21
    float t1 = dht1.readTemperature();
    float h1 = dht1.readHumidity();
    float t2 = dht2.readTemperature();
    float h2 = dht2.readHumidity();

    // publish data to MQTT broker
    char t1Char[6];
    char h1Char[6];
    char t2Char[6];
    char h2Char[6];
    dtostrf(t1, 4, 2, t1Char);
    dtostrf(h1, 4, 2, h1Char);
    dtostrf(t2, 4, 2, t2Char);
    dtostrf(h2, 4, 2, h2Char);
    client.publish("gm/dev/rice/temp1", t1Char);
    client.publish("gm/dev/rice/hum1", h1Char);
    client.publish("gm/dev/rice/temp2", t2Char);
    client.publish("gm/dev/rice/hum2", h2Char);

    // // read voltage from sensor
    // float v1 = calcVoltage(VOLTAGE_METER_PIN1);
    // float v2 = calcVoltage(VOLTAGE_METER_PIN2);

    // Serial.print("Voltage 1: ");
    // Serial.println(v1);
    // Serial.print("Voltage 2: ");
    // Serial.println(v2);

    lastPublishTime = millis();
  }

  // check if we need to turn on/off the heater
  if (heatStatus == true)
  {
    if (dht1.readTemperature() > maxTemp)
    {
      digitalWrite(HEAT_RELAY, LOW);
      heatRelay = false;
      client.publish("gm/dev/rice/heatRelay", "off");
    }
    if (dht1.readTemperature() < maxTemp - 10)
    {
      digitalWrite(HEAT_RELAY, HIGH);
      heatRelay = true;
      client.publish("gm/dev/rice/heatRelay", "on");
    }
  }
}

float calcVoltage(int pin)
{
  // Read the voltage from the sensor
  int sensorValue = analogRead(pin);

  // Convert the sensor value to voltage
  float voltage = sensorValue * (3.3 / 4095.0) * 5.5;
  return voltage;
}

void callback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == "gm/dev/rice/control/fan")
  {
    if (messageTemp == "true")
    {
      if (fanRelay == false)
      {
        fanRelay = true;
        digitalWrite(FAN_RELAY, HIGH);
        client.publish("gm/dev/rice/fanState", "on");
      }

      else
      {
        fanRelay = false;
        digitalWrite(FAN_RELAY, LOW);
        client.publish("gm/dev/rice/fanState", "off");
      }
    }
  }

  if (String(topic) == "gm/dev/rice/control/heat")
  {
    if (messageTemp == "true")
    {
      if (heatStatus == false)
      {
        heatRelay = true;
        heatStatus = true;
        digitalWrite(HEAT_RELAY, HIGH);
        client.publish("gm/dev/rice/heatState", "on");
        client.publish("gm/dev/rice/heatRelay", "on");
      }

      else
      {
        heatRelay = false;
        heatStatus = false;
        digitalWrite(HEAT_RELAY, LOW);
        client.publish("gm/dev/rice/heatState", "off");
      }
    }
  }

  if (String(topic) == "gm/dev/rice/control/maxHeat")
  {
    Serial.print("Changing max temp to ");
    Serial.println(messageTemp);
    maxTemp = messageTemp.toFloat();
  }

  if (String(topic) == "gm/dev/rice/control/off")
  {
    if (messageTemp == "true")
    {
      heatRelay = false;
      heatStatus = false;
      digitalWrite(HEAT_RELAY, LOW);

      fanRelay = false;
      digitalWrite(FAN_RELAY, LOW);

      client.publish("gm/dev/rice/heatState", "off");
      client.publish("gm/dev/rice/fanState", "off");
      client.publish("gm/dev/rice/heatRelay", "off");
      client.publish("gm/dev/rice/offState", "finished");
    }
  }

  if (String(topic) == "gm/dev/rice/control/start")
  {
    if (messageTemp == "true")
    {
      heatStatus = true;
      digitalWrite(HEAT_RELAY, HIGH);

      fanRelay = true;
      digitalWrite(FAN_RELAY, HIGH);

      client.publish("gm/dev/rice/heatState", "on");
      client.publish("gm/dev/rice/fanState", "on");
      client.publish("gm/dev/rice/heatRelay", "on");
      client.publish("gm/dev/rice/startState", "started");
    }
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqttClientId, mqttUser, mqttPassword))
    {
      Serial.println("connected");
      // Subscribe
      client.subscribe("gm/dev/rice/control/#");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}