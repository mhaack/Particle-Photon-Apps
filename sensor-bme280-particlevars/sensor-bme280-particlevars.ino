/***************************************************************************
  This app gets BME280 humidity, temperature & pressure sensor data and
  exposed them as variables to the Particle cloud.

  Dependecies:
  * Adafruit BME280 library (https://github.com/mhaack/Adafruit_BME280_Library)

  Author:
  Markus Haack <mh.devx@gmail.com>

  The app connects to the Sensor using I2C. If you would like to wire using SPI
  check the Adafruit BME280 library test application.
 ***************************************************************************/

#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

double temperature = 0;
double pressure = 0;
double altitude = 0;
double humidity = 0;


void setup() {
  Serial.begin(9600);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Spark.variable("temperature", &temperature, DOUBLE);
  Spark.variable("pressure", &pressure, DOUBLE);
  Spark.variable("altitude", &altitude, DOUBLE);
  Spark.variable("humidity", &humidity, DOUBLE);
}

void loop() {
  temperature = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidity = bme.readHumidity();

  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(altitude);
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.println();
  delay(5000);
}
