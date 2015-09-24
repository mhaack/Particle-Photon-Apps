#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "SparkFunPhant.h"
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

// moisture sensor
const int soilPower = D4;
const int soilSensor1 = A0;
int soil1 = 0;

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
  pinMode(soilSensor1,INPUT);
  pinMode(soilPower, OUTPUT);
  digitalWrite(soilPower, LOW);

  Serial.begin(9600);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // declare Particle cloud variables
  Particle.variable("temperature", &temperature, DOUBLE);
  Particle.variable("pressure", &pressure, DOUBLE);
  Particle.variable("humidity", &humidity, DOUBLE);
  Particle.variable("soil1", &soil1, INT);
}

void loop() {
    int sensorStatus = 1;
    if (lastMeasurement > MEASUREMENT_RATE) {
      // read sensor data
      sensorStatus = readSensorData();
      if (sensorStatus >= 0) {
          lastMeasurement = 0;
          postToParticle(sensorStatus); // always publish event
          dumpSerial();
      }
    }

    if (lastPost > POST_RATE || sensorStatus > 1) {
      while (postToPhant(sensorStatus) <= 0) { // publish to phant
        Serial.println("Phant post failed. Trying again.");
				// Delay 1s, so we don't flood the server. Little delay's allow the Photon time
				// to communicate with the Cloud.
        for (int i=0; i<1000; i++) {
              delay(1);
        }
      }
      lastPost = 0;
    }
}

int postToParticle(int status) {
  char publishString[128];
  sprintf(publishString,"{\"status\": %d, \"temp\": %0.2f, \"pressure\": %0.2f, \"humidity\": %0.2f, , \"soil1\": %u}", status, temperature, pressure, humidity, soil1);
  Particle.publish("sensor",publishString);
  return 1;
}

int postToPhant(int status) {
  // add variables
  phant.add("status", status);
  phant.add("temp", temperature, 2);
  phant.add("pressure", pressure, 2);
  phant.add("humidity", humidity, 2);
  phant.add("soil1", soil1);

  return phant.particlePost();
}


byte readSensorData() {
  byte status = readBMESensor();

  soil1 = readSoilSensor(soilSensor1);

  // todo define thresholds for dry, wet, max increase/decrease
  // todo integrate the soil sensor value into status
  return status;
}

// read BME280 sensor data
byte readBMESensor() {
  double currentTemperature = 0;
  digitalWrite(led, HIGH);
  currentTemperature = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidity = bme.readHumidity();
  delay(200);
  digitalWrite(led, LOW);

  // calc temp diff for changes at .x level
  int tempDiff = (currentTemperature - temperature)*10;
  temperature = currentTemperature;

  if (tempDiff > 0) {
    return 1; // indicate temp increase above threshold
  } else if (tempDiff < 0){
    return 2; // indicate temp decrease above threshold
  } else {
    return 0; // indicate normal reading
  }
}

// read soil sensor data
int readSoilSensor(int soilSensor) {
    unsigned int val = 0;
    digitalWrite(led, HIGH);
    digitalWrite(soilPower, HIGH); //turn sensor power on
    delay(10);
    val = analogRead(soilSensor);
    digitalWrite(soilPower, LOW); //turn sensor power off
    delay(190);
    digitalWrite(led, LOW);
    return val;
}

void dumpSerial() {
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

  Serial.print("Soil 1 = ");
  Serial.println(soil1);
}
