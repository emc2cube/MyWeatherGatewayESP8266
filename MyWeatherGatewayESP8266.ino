/**
   Important: I2C setup and NRF24L01 wiring
   By default the NRF24 CE pin is wired on GPIO4. On ESP8266 default SDA and SCL pin are respectively GPIO4 and GPIO5.
   Both CE pin and SDA (or even SCL) can be moved so depending of your setup you will have to move at least one of these.
   Look for "#define MY_RF24_CE_PIN", "#define SDA" and "#define SCL" lines to adapt to your wiring.

   By default Adafruit BME280 library is looking for a BME280 sensor on the I2C bus at (0x77).
   If your sensor is set with the low adress (0x76) you can uncomment the lines "#undef BME280_ADDRESS" and "#define BME280_ADDRESS (0x76)"
   Or you can set it on high by changing the jumper/bridge setting on the PCB.

   Make sure to fill in your ssid and WiFi password below for ssid & pass.
*/

// ----------------------------------------------------------------------------
// MySensor ESP8266 Gateway configuration
// ----------------------------------------------------------------------------
#include <EEPROM.h>
#include <SPI.h>

// Enable debug prints to serial monitor
//#define MY_DEBUG    // I2C devices may have a funky behaviour when serial communication is occuring and GPIO 1 and 3 are used for I2C bus (RX and TX ports).

// Use a bit lower baudrate for serial prints on ESP8266 than default in MyConfig.h
#define MY_BAUD_RATE 9600

// Enables and select radio type (if attached)
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Gateway mode always enabled for ESP8266. But we add this anyway ;)
#define MY_GATEWAY_ESP8266

#define MY_ESP8266_SSID "MySSID"
#define MY_ESP8266_PASSWORD "MyVerySecretPassword"

// To use native I2C pins CE should be moved. Declare the new GPIO used here.
// GPIO 3 is pin D9 (RX below D8)
//#define MY_RF24_CE_PIN 3

// Enable UDP communication
//#define MY_USE_UDP

// Enable MY_IP_ADDRESS here if you want a static ip address (no DHCP)
//#define MY_IP_ADDRESS 192,168,178,87

// If using static ip you need to define Gateway and Subnet address as well
#define MY_IP_GATEWAY_ADDRESS 192,168,178,1
#define MY_IP_SUBNET_ADDRESS 255,255,255,0

// The port to keep open on node server mode
#define MY_PORT 5003

// How many clients should be able to connect to this gateway (default 1)
#define MY_GATEWAY_MAX_CLIENTS 2

// Controller ip address. Enables client mode (default is "server" mode).
// Also enable this if MY_USE_UDP is used and you want sensor data sent somewhere.
//#define MY_CONTROLLER_IP_ADDRESS 192, 168, 178, 68

// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE

// Enable Inclusion mode button on gateway
// #define MY_INCLUSION_BUTTON_FEATURE
// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
#define MY_INCLUSION_MODE_BUTTON_PIN  3


// Flash leds on rx/tx/err
// #define MY_LEDS_BLINKING_FEATURE
// Set blinking period
// #define MY_DEFAULT_LED_BLINK_PERIOD 300

// Led pins used if blinking feature is enabled above
#define MY_DEFAULT_ERR_LED_PIN 16  // Error led pin
#define MY_DEFAULT_RX_LED_PIN  16  // Receive led pin
#define MY_DEFAULT_TX_LED_PIN  16  // the PCB, on board LED

#if defined(MY_USE_UDP)
#include <WiFiUDP.h>
#else
#include <ESP8266WiFi.h>
#endif

#include <MySensor.h>

// ----------------------------------------------------------------------------
// Child sensor ids
// ----------------------------------------------------------------------------
#define BARO_CHILD 0
#define TEMP_CHILD 1
#define HUM_CHILD 2

// ----------------------------------------------------------------------------
// I2C setup: SDA and SCL pin selection
// ----------------------------------------------------------------------------
#include <Wire.h>

// If using the standard MySensor wiring direction we can't use default I2C ports. I2C ports moved next to the second GND and 3.3v ports for convenient wiring of I2c modules.
#define SDA 3  //default to GPIO 4 (D2) but NRF24L01+ CE is plugged on D2 by default for the ESP gateway. SDA changed to GPIO 3 (RX)
#define SCL 1  //default to GPIO 5 (D1). SDA changed to GPIO 1 (TX)

// ----------------------------------------------------------------------------
// BME280 libraries and variables
// Written by Limor Fried & Kevin Townsend for Adafruit Industries.
// https://github.com/adafruit/Adafruit_BME280_Library
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//#undef BME280_ADDRESS         // Undef BME280_ADDRESS from the BME280 library to easily override I2C address
//#define BME280_ADDRESS (0x76) // Low = 0x76 , High = 0x77 (default on adafruit and sparkfun BME280 modules, default for library)

Adafruit_BME280 bme; // Use I2C

// ----------------------------------------------------------------------------
// Weather station variables and functions
// ----------------------------------------------------------------------------
// Adapt this constant: set it to the altitude above sealevel at your home location.
const float ALTITUDE = 23; // meters above sea level. Use your smartphone GPS to get an accurate value!

// Set this to true if you want to send values altough the values did not change.
// This is only recommended when not running on batteries.
const bool SEND_ALWAYS = true;

//////////////////////////////////////////////////////////
// You should not need to edit anything after this line //
//////////////////////////////////////////////////////////

// Constant for the world wide average pressure
const float SEALEVEL_PRESSURE = 1013.25;

// Sleep time between reads (in ms). Do not change this value as the forecast algorithm needs a sample every minute.
const unsigned long SLEEP_TIME = 60000;
unsigned long previousMillis = 0;

const char *weatherStrings[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
  STABLE = 0,       // Stable weather
  SUNNY = 1,        // Slowly rising HP stable good weather
  CLOUDY = 2,       // Slowly falling Low Pressure System, stable rainy weather
  UNSTABLE = 3,     // Quickly rising HP, not stable weather
  THUNDERSTORM = 4, // Quickly falling LP, Thunderstorm, not stable
  UNKNOWN = 5       // Unknown, more time needed
};

const char *situationStrings[] = { "very low", "low", "normal", "high", "very high" };
enum WEATHER_SITUATION
{
  VERY_LOW_PRESSURE = 0,   // p > -7.5hPa
  LOW_PRESSURE = 1,        // p > -2.5hPa
  NORMAL_PRESSURE = 2,     // p < +/-2.5hPa
  HIGH_PRESSURE = 3,       // p > +2.5hPa
  VERY_HIGH_PRESSURE = 4,  // p > +7.5hPa
};

float lastPressure = -1;
float lastTemp = -1;
float lastHum = -1;
int lastForecast = -1;
int lastSituation = NORMAL_PRESSURE;

const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];

// this CONVERSION_FACTOR is used to convert from Pa to kPa in the forecast algorithm
// get kPa/h by dividing hPa by 10
#define CONVERSION_FACTOR (1.0/10.0)

int minuteCount = 0;
bool firstRound = true;
// average value is used in forecast algorithm.
float pressureAvg;
// average after 2 hours is used as reference value for the next iteration.
float pressureAvg2;

float dP_dt;
boolean metric;
MyMessage tempMsg(TEMP_CHILD, V_TEMP);
MyMessage humMsg(HUM_CHILD, V_HUM);
MyMessage pressureMsg(BARO_CHILD, V_PRESSURE);
MyMessage forecastMsg(BARO_CHILD, V_FORECAST);
MyMessage situationMsg(BARO_CHILD, V_VAR1);
MyMessage forecastMsg2(BARO_CHILD, V_VAR2);


void initPressureSensor() {
  Serial.print(F("NRF24L01+ CE pin on GPIO"));
  Serial.println(MY_RF24_CE_PIN);
#if defined(SDA) && defined(SCL)
  Serial.print(F("Using custom I2C pins: SDA on GPIO"));
  Serial.print(SDA);
  Serial.print(F(" and SCL on GPIO"));
  Serial.println(SCL);
  Wire.begin(SDA, SCL);
#else
  Serial.println(F("Using default I2C pins: SDA on GPIO4 and SCL on GPIO5"));
  Wire.begin();
#endif
  if (!bme.begin(BME280_ADDRESS)) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring or I2C address!"));
    while (1) {
      yield();
    }
  }
}


int getWeatherSituation(float pressure)
{
  int situation = NORMAL_PRESSURE;

  float delta = pressure - SEALEVEL_PRESSURE;
  if (delta > 7.5)
  {
    situation = VERY_HIGH_PRESSURE;
  }
  else if (delta > 2.5)
  {
    situation = HIGH_PRESSURE;
  }
  else if (delta < -7.5)
  {
    situation = VERY_LOW_PRESSURE;
  }
  else if (delta < -2.5)
  {
    situation = LOW_PRESSURE;
  }
  else
  {
    situation = NORMAL_PRESSURE;
  }

  return situation;
}


float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
  {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }

  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  // Uncomment when debugging
  Serial.print(F("### 5min-Average:"));
  Serial.print(lastPressureSamplesAverage);
  Serial.println(F(" hPa"));

  return lastPressureSamplesAverage;
}


// Algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure)
{
  // Calculate the average of the last n minutes.
  int index = minuteCount % LAST_SAMPLES_COUNT;
  lastPressureSamples[index] = pressure;

  minuteCount++;
  if (minuteCount > 185)
  {
    minuteCount = 6;
  }

  if (minuteCount == 5)
  {
    pressureAvg = getLastPressureSamplesAverage();
  }
  else if (minuteCount == 35)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change * 2; // note this is for t = 0.5hour
    }
    else
    {
      dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
    }
  }
  else if (minuteCount == 65)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) //first time initial 3 hour
    {
      dP_dt = change; //note this is for t = 1 hour
    }
    else
    {
      dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 95)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 1.5; // note this is for t = 1.5 hour
    }
    else
    {
      dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 125)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    pressureAvg2 = lastPressureAvg; // store for later use.
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2; // note this is for t = 2 hour
    }
    else
    {
      dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 155)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2.5; // note this is for t = 2.5 hour
    }
    else
    {
      dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 185)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 3; // note this is for t = 3 hour
    }
    else
    {
      dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
    }
    pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

  int forecast = UNKNOWN;
  if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
  {
    forecast = UNKNOWN;
  }
  else if (dP_dt < (-0.25))
  {
    forecast = THUNDERSTORM;
  }
  else if (dP_dt > 0.25)
  {
    forecast = UNSTABLE;
  }
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
  {
    forecast = CLOUDY;
  }
  else if ((dP_dt > 0.05) && (dP_dt < 0.25))
  {
    forecast = SUNNY;
  }
  else if ((dP_dt > (-0.05)) && (dP_dt < 0.05))
  {
    forecast = STABLE;
  }
  else
  {
    forecast = UNKNOWN;
  }

  // Uncomment when dubugging
  Serial.print(F("Forecast at minute "));
  Serial.print(minuteCount);
  Serial.print(F(" dP/dt = "));
  Serial.print(dP_dt);
  Serial.print(F("kPa/h --> "));
  Serial.println(weatherStrings[forecast]);

  return forecast;
}


bool updatePressureSensor()
{
  bool changed = false;

  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure_local = bme.readPressure() / 100.0; // Read atmospheric pressure at local altitude
  float pressure = ( pressure_local / pow((1.0 - ( ALTITUDE / 44330.0 )), 5.255)); // Local pressure adjusted to sea level pressure using user altitude

  if (!metric)
  {
    // Convert to fahrenheit
    temperature = temperature * 9.0 / 5.0 + 32.0;
  }

  int forecast = sample(pressure);
  int situation = getWeatherSituation(pressure);

  if (SEND_ALWAYS || (temperature != lastTemp))
  {
    changed = true;
    lastTemp = temperature;
    Serial.print(F("Temperature = "));
    Serial.print(temperature);
    Serial.println(metric ? F(" *C") : F(" *F"));
    if (!send(tempMsg.set(lastTemp, 1)))
    {
      lastTemp = -1.0;
    }
  }

  if (SEND_ALWAYS || (humidity != lastHum))
  {
    lastHum = humidity;
    changed = true;
    Serial.print(F("Humidity = "));
    Serial.print(humidity);
    Serial.println(F(" %"));
    if (!send(humMsg.set(lastHum, 1)))
    {
      lastHum = -1.0;
    }
  }

  if (SEND_ALWAYS || (pressure != lastPressure))
  {
    changed = true;
    lastPressure = pressure;
    Serial.print(F("Sea level Pressure = "));
    Serial.print(pressure);
    Serial.println(F(" hPa"));
    if (!send(pressureMsg.set(lastPressure, 1)))
    {
      lastPressure = -1.0;
    }
  }

  if (SEND_ALWAYS || (forecast != lastForecast))
  {
    changed = true;
    lastForecast = forecast;
    Serial.print(F("Forecast = "));
    Serial.println(weatherStrings[forecast]);
    if (send(forecastMsg.set(weatherStrings[lastForecast])))
    {
      if (!send(forecastMsg2.set(lastForecast)))
      {
      }
    }
    else
    {
      lastForecast = -1.0;
    }
  }

  if (SEND_ALWAYS || (situation != lastSituation))
  {
    changed = true;
    lastSituation = situation;
    Serial.print(F("Situation = "));
    Serial.println(situationStrings[situation]);
    if (!send(situationMsg.set(lastSituation)))
    {
      lastSituation = -1.0;
    }
  }

  return changed;
}


void setup() {
  // Setup locally attached sensors
  initPressureSensor();
  metric = getConfig().isMetric;
}


void presentation() {
  // Present locally attached sensors

  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Weather Station Gateway", "1.6");

  // Register sensors to gw (they will be created as child devices)
  present(BARO_CHILD, S_BARO);
  present(TEMP_CHILD, S_TEMP);
  present(HUM_CHILD, S_HUM);
}


void loop() {
  // Send locally attached sensor data here
  updatePressureSensor();
  wait(SLEEP_TIME);   // do not use sleep() or delay(), it would prevent required TCP/IP and MySensor operations!
}

