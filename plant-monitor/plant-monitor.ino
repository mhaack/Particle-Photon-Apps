#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "SparkFun-Spark-Phant.h"
#include "elapsedMillis.h"

#define SEALEVELPRESSURE_HPA (1013.25)

// control led
const int led = D7;

// BME280 sensor
Adafruit_BME280 bme;
double temperature = 0;
double pressure = 0;
double altitude = 0;
double humidity = 0;

// SparkFun phant library
const char server[] = "data.sparkfun.com";
const char publicKey[] = "2JLLbmQoYEsKVxjrQV3a";
const char privateKey[] = "GPxxAGzknDsN4AlwP4vj";
Phant phant(server, publicKey, privateKey);

// interval counters for measurement and post
const int MEASUREMENT_RATE = 30000; // read sensor data every 30 sec
const int POST_RATE = 15 * 60000; // post data every 15 minutes
elapsedMillis lastMeasurement;
elapsedMillis lastPost;

void setup()
{
  pinMode(led, OUTPUT);
  Serial.begin(9600);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Particle.variable("temperature", &temperature, DOUBLE);
  Particle.variable("pressure", &pressure, DOUBLE);
  Particle.variable("humidity", &humidity, DOUBLE);
}

void loop() {
    int sensorStatus = 1;
    if (lastMeasurement > MEASUREMENT_RATE) {
      // read sensor data
      sensorStatus = readSensorData();
      if (sensorStatus > 0) {
          lastMeasurement = 0;
          postToParticle(sensorStatus); // always publish event
      }
    }

    if (lastPost > POST_RATE || sensorStatus == 2) {
      if (postToPhant(sensorStatus) > 0) { // publish to phant
        lastPost = 0;
      }
    }
}

int postToParticle(int status) {
  char publishString[128];
  sprintf(publishString,"{\"status\": \"%d\", \"temp\": \"%0.2f\", \"pressure\": %0.2f, \"humidity\": %0.2f}",status, temperature, pressure, humidity);
  Particle.publish("sensor",publishString);
  return 1;
}

int postToPhant(int status) {
  // add variables
  phant.add("status", status);
  phant.add("temp", temperature);
  phant.add("pressure", pressure);
  phant.add("humidity", humidity);

  // post data
  TCPClient client;
  char response[512];
  int i = 0;
  int retVal = 0;

  if (client.connect(server, 80)) {
      Serial.println("Posting!");

      client.print(phant.post());
      delay(1000);
      while (client.available())      {
          char c = client.read();
          if (i < 512) {
              response[i++] = c;
            }
      }
      if (strstr(response, "200 OK"))      {
          Serial.println("Post success!");
          retVal = 1;
      } else if (strstr(response, "400 Bad Request")) {
          Serial.println("Bad request");
          retVal = -1;
      } else {
          retVal = -2;
      }
  } else {
      Serial.println("connection failed");
      retVal = -3;
  }
  client.stop();
  return retVal;
}

int readSensorData() {
  double currentTemperature = 0;
  digitalWrite(led, HIGH);

  currentTemperature = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidity = bme.readHumidity();
  delay(100);
  digitalWrite(led, LOW);

  Serial.print("Temperature = ");
  Serial.print(currentTemperature);
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

  // fload abs (std::abs) requiere #include <cmath>
  //double tempDiff = std::abs (currentTemperature - temperature);
  int tempDiff = abs ((currentTemperature - temperature)*10);
  temperature = currentTemperature;

  if (tempDiff < 1) {
    return 1;
  } else {
    return 2;
  }
}
