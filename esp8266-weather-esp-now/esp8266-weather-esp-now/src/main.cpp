/*
 ESP-NOW based sensor using a BME280 temperature/pressure/humidity sensor
 Sends readings every 15 minutes to a server with a fixed mac address
 It takes about 215 milliseconds to wakeup, send a reading and go back to sleep, 
 and it uses about 70 milliAmps while awake and about 25 microamps while sleeping, 
 so it should last for a good year even AAA alkaline batteries. 
 Anthony Elder
 License: Apache License v2
*/
#include <Arduino.h>

#include <ESP8266WiFi.h>
extern "C"
{
#include <espnow.h>
}

#define SEALEVELPRESSURE_HPA (1027)
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "SdsDustSensor.h"

Adafruit_BME280 bme; // I2C
int rxPin = D6;
int txPin = D7;
SdsDustSensor sds(rxPin, txPin);

// this is the MAC Address of the remote ESP server which receives these sensor readings
uint8_t remoteMac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};

#define WIFI_CHANNEL 4
#define SLEEP_SECS 5 * 60    // 5 minutes
#define SEND_TIMEOUT 245 // 245 millis seconds timeout

// keep in sync with slave struct
struct __attribute__((packed)) SENSOR_DATA
{
    float temp;
    float humidity;
    float pressure;
    float pm25;
    float pm10;
} sensorData;

volatile boolean callbackCalled;
int error;
float pm10, pm25;

void readBME280()
{
    PmResult pm = sds.readPm();
    if (pm.isOk())
    {
        Serial.print("PM2.5 = ");
        Serial.print(pm.pm25);
        Serial.print(", PM10 = ");
        Serial.println(pm.pm10);
        sensorData.pm10 = pm.pm10;
        sensorData.pm25 = pm.pm25;

        // if you want to just print the measured values, you can use toString() method as well
        Serial.println(pm.toString());
    }
    else
    {
        // notice that loop delay is set to 0.5s and some reads are not available
        Serial.print("Could not read values from sensor, reason: ");
        Serial.println(pm.statusToString());
    }
    sensorData.temp = bme.readTemperature();
    sensorData.humidity = bme.readHumidity();
    sensorData.pressure = bme.readPressure() / 100.0F;

    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}

void gotoSleep()
{
    // add some randomness to avoid collisions with multiple devices
    int sleepSecs = SLEEP_SECS + ((uint8_t)RANDOM_REG32 / 2);
    Serial.printf("Up for %i ms, going to sleep for %i secs...\n", millis(), sleepSecs);
    ESP.deepSleep(sleepSecs * 1000000, RF_NO_CAL);
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
    sds.begin();

    Serial.println(sds.queryFirmwareVersion().toString());       // prints firmware version
    Serial.println(sds.setActiveReportingMode().toString());     // ensures sensor is in 'active' reporting mode
    Serial.println(sds.setContinuousWorkingPeriod().toString()); // ensures sensor has continuous working period - default but not recommended
    Serial.println();

    // read sensor first before awake generates heat
    readBME280();

    WiFi.mode(WIFI_STA); // Station mode for esp-now sensor node
    WiFi.disconnect();

    Serial.printf("This mac: %s, ", WiFi.macAddress().c_str());
    Serial.printf("target mac: %02x%02x%02x%02x%02x%02x", remoteMac[0], remoteMac[1], remoteMac[2], remoteMac[3], remoteMac[4], remoteMac[5]);
    Serial.printf(", channel: %i\n", WIFI_CHANNEL);

    if (esp_now_init() != 0)
    {
        Serial.println("*** ESP_Now init failed");
        gotoSleep();
    }

    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_add_peer(remoteMac, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);

    esp_now_register_send_cb([](uint8_t *mac, uint8_t sendStatus) {
        Serial.printf("send_cb, send done, status = %i\n", sendStatus);
        callbackCalled = true;
    });

    callbackCalled = false;

    uint8_t bs[sizeof(sensorData)];
    memcpy(bs, &sensorData, sizeof(sensorData));
    esp_now_send(NULL, bs, sizeof(sensorData)); // NULL means send to all peers
}

void loop()
{
    if (callbackCalled || (millis() > SEND_TIMEOUT))
    {
        gotoSleep();
    }
}
