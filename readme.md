# FreeRTOS ESP8266 Aircon remote for Toshiba Air Conditioner
This program listens to an MQTT queue for instructions, when a byte is received on the queue it is sent via IR to the air conditioning unit.
This means that construction of the byte to be sent must be done by the application putting it into the queue, for me this is done by an iPhone app I built to control the aircon.

Additionally this also queries a BME280 Temperature and Barometric pressure sensor and publishes the results into an mqtt queue that can then be read by Telegrafs MQTT Listener so that Temperature & Pressure can be graphed over time or reported by your control app to show the difference between current & set temperature.

This project makes use of [Baoshi's Paho client code for MQTT](https://github.com/baoshi/ESP-RTOS-Paho), [Baoshi's i2c driver](https://github.com/baoshi/ESP-I2C-OLED) and [RyAndrews BME280 driver](https://github.com/RyAndrew/esp8266_i2c_bme280) which I adapted to work with Baoshi's FreeRTOS i2c code.