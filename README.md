MyWeatherGatewayESP8266
=======================

MySensor WiFi Gateway with BME280 sensor.


Description:
------------

Allow your MySensor Gateway to collect weather information (barometric pressure) and indoor temperature ans humidity.

Use a BME280 sensor. This is probably an option slighly more expensive that DHT22 + Barometric pressure such as BMP180, but end up in a smaller form factor and less wiring (available GPIO is quite limited on the node mcu once the radion is wired)

Because of the default wiring of the radio, you will need either to move the CE pin, or to customize wich ports will be used as I2C bus.


Use MySensor 1.6 branch https://github.com/mysensors/Arduino and Adafruit BME280 library https://github.com/adafruit/Adafruit_BME280_Library

Based on the WeatherStationSensor https://github.com/mysensors/Arduino/tree/development/libraries/MySensors/examples/WeatherStationSensor and GatewayESP8266 https://github.com/mysensors/Arduino/tree/development/libraries/MySensors/examples/GatewayESP8266