
/**
 * ESP-NOW to Watson IoT Gateway Example 
 * 
 * This shows how to use an ESP8266/Arduino as a Gateway device on the Watson 
 * IoT Platform enabling remote ESP-NOW devices to be Watson IoT Devices.
 * 
 * Author: Anthony Elder
 * License: Apache License v2
 */

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
extern "C"
{
#include <espnow.h>
#include "user_interface.h"
}

Adafruit_BME280 bme; // I2C
float temp_inside, humidity_inside, pressure_inside;

//-------- Customise these values -----------
const char *ssid = "<YOUR WIFI SSID>";
const char *password = "<YOUR WIFI PASSWORD>";

#define DEVICE_TYPE "esp8266"
#define DEVICE_ID "espindoor"
//-------- Customise the above values --------

/* Set a private Mac Address
 *  http://serverfault.com/questions/40712/what-range-of-mac-addresses-can-i-safely-use-for-my-virtual-machines
 * Note: the point of setting a specific MAC is so you can replace this Gateway ESP8266 device with a new one
 * and the new gateway will still pick up the remote sensors which are still sending to the old MAC 
 */
uint8_t mac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};
void initVariant()
{
  WiFi.mode(WIFI_AP);
  wifi_set_macaddr(SOFTAP_IF, &mac[0]);
}

char server[] = "<YOUR MQTT SERVER IP>";
const String clientId = "g:" DEVICE_TYPE ":" DEVICE_ID;
const String serverUsername = "<YOUR MQTT SERVER USERNAME>";
const String serverPassword = "<YOUR MQTT SERVER PASSWORD>";

// the X's get replaced with the remote sensor device mac address
const char deviceTopic[] = "o/weather";
const char officeTopic[] = "r2/weather";

WiFiClient wifiClient;
PubSubClient client(server, 1883, wifiClient);

String deviceMac;

// keep in sync with ESP_NOW sensor struct
struct __attribute__((packed)) SENSOR_DATA
{
  float temp;
  float humidity;
  float pressure;
  float pm25;
  float pm10;

} sensorData;

volatile boolean haveReading = false;

/* Presently it doesn't seem posible to use both WiFi and ESP-NOW at the
 * same time. This gateway gets around that be starting just ESP-NOW and
 * when a message is received switching on WiFi to sending the MQTT message
 * to Watson, and then restarting the ESP. The restart is necessary as 
 * ESP-NOW doesn't seem to work again even after WiFi is disabled.
 * Hopefully this will get fixed in the ESP SDK at some point.
 */

void initEspNow()
{
  if (esp_now_init() != 0)
  {
    Serial.println("*** ESP_Now init failed");
    ESP.restart();
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  esp_now_register_recv_cb([](uint8_t *mac, uint8_t *data, uint8_t len) {
    deviceMac = "";
    deviceMac += String(mac[0], HEX);
    deviceMac += String(mac[1], HEX);
    deviceMac += String(mac[2], HEX);
    deviceMac += String(mac[3], HEX);
    deviceMac += String(mac[4], HEX);
    deviceMac += String(mac[5], HEX);

    memcpy(&sensorData, data, sizeof(sensorData));

    Serial.print("recv_cb, msg from device: ");
    Serial.print(deviceMac);
    Serial.printf(" Temp=%0.1f, Hum=%0.1f, pressure=%0.1fmb, pm10=%0.1f, pm2.5=%0.1f\n",
                  sensorData.temp, sensorData.humidity, sensorData.pressure, sensorData.pm10, sensorData.pm25);

    haveReading = true;
  });
}

void setup()
{
  if (!bme.begin(0x76, &Wire))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }

  Serial.begin(115200);
  Serial.println();

  Serial.print("This node AP mac: ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.print("This node STA mac: ");
  Serial.println(WiFi.macAddress());

  initEspNow();
}

int heartBeat;

void publishTo(const char *topic, const char *payload)
{
  Serial.print("publish ");
  if (client.publish(topic, payload))
  {
    Serial.print(" OK ");
  }
  else
  {
    Serial.print(" FAILED ");
  }
  Serial.print(" topic: ");
  Serial.print(topic);
  Serial.print(" payload: ");
  Serial.println(payload);
}

void sendToWatson()
{

  String payload1 = "{";
  payload1 += "\"temperature\":" + String(sensorData.temp, 1) + ",";
  payload1 += "\"humidity\":" + String(sensorData.humidity, 1) + ",";
  payload1 += "\"pressure\":" + String(sensorData.pressure, 1) + ",";
  payload1 += "\"pm25\":" + String(sensorData.pm25, 1) + ",";
  payload1 += "\"pm10\":" + String(sensorData.pm10, 1);
  payload1 += "}";

  String payload2 = "{";
  payload2 += "\"temperature\":" + String(bme.readTemperature(), 1) + ",";
  payload2 += "\"humidity\":" + String(bme.readHumidity(), 1) + ",";
  payload2 += "\"pressure\":" + String(bme.readPressure() / 100.0F, 1);
  payload2 += "}";

  String topic1 = String(deviceTopic);
  String topic2 = String(officeTopic);
  publishTo(topic1.c_str(), payload1.c_str());
  publishTo(topic2.c_str(), payload2.c_str());
}

void wifiConnect()
{
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(250);
    Serial.print(".");
  }
  Serial.print("\nWiFi connected, IP address: ");
  Serial.println(WiFi.localIP());
}

void mqttConnect()
{
  if (!!!client.connected())
  {
    Serial.print("Reconnecting MQTT client to ");
    Serial.println(server);
    while (!!!client.connect(clientId.c_str(), serverUsername.c_str(), serverPassword.c_str()))
    {
      Serial.print(".");
      delay(250);
    }
    Serial.println();
  }
}

void loop()
{
  if (millis() - heartBeat > 30000)
  {
    Serial.println("Waiting for ESP-NOW messages...");
    heartBeat = millis();
  }

  if (haveReading)
  {
    haveReading = false;
    wifiConnect();
    mqttConnect();
    sendToWatson();
    client.disconnect();
    delay(200);
    ESP.restart(); // <----- Reboots to re-enable ESP-NOW
  }
}
