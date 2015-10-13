#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "SparkFunPhant.h"
#include "elapsedMillis.h"

#define SEALEVELPRESSURE_HPA (1013.25)

// control led
const int led = D7;

// main status
int lastSensorStatus = 0, currentSensorStatus = 0;
char publishString[128];

// BME280 sensor
Adafruit_BME280 bme;
double temperature = 0;
double pressure = 0;
double altitude = 0;
double humidity = 0;

// moisture sensor
const int soildThresholdLow = 500;
const int soildThresholdHigh = 3500;
struct SoilSensor {
  int sensor;
  int power;
};
SoilSensor soilSensors[3] = {{ A0, D4 }, { A1, D5 }, { A2, D6 }};
int soil1 = 2048, soil2 = 2048, soil3 = 2048;

// SparkFun phant library
const char server[] = "data.sparkfun.com";
const char publicKey[] = "2JLLbmQoYEsKVxjrQV3a";
const char privateKey[] = "GPxxAGzknDsN4AlwP4vj";
Phant phant(server, publicKey, privateKey);

// interval counters for measurement and post
const unsigned int MEASUREMENT_RATE = 60000; // read sensor data every 60 sec
const unsigned int POST_RATE = 15 * 60000; // post data every 15 minutes
elapsedMillis lastMeasurement;
elapsedMillis lastPost;

void setup() {
  Serial.begin(9600);

  pinMode(led, OUTPUT);
  for (unsigned int i = 0; i < sizeof(soilSensors); i++) {
    pinMode(soilSensors[i].sensor, INPUT);
    pinMode(soilSensors[i].power, OUTPUT);
  }

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // declare Particle cloud variables
  Particle.variable("temperature", &temperature, DOUBLE);
  Particle.variable("pressure", &pressure, DOUBLE);
  Particle.variable("humidity", &humidity, DOUBLE);
  Particle.variable("soil1", &soil1, INT);
  Particle.variable("soil2", &soil2, INT);
  Particle.variable("soil3", &soil3, INT);
}

void loop() {
    // read sensor data
    if (lastMeasurement > MEASUREMENT_RATE) {
      int bmeSensorStatus = readBMESensor();
      delay(200);
      int soil1SensorStatus = readSoilSensor(soilSensors[0].sensor, soilSensors[0].power, soil1);
      delay(200);
      int soil2SensorStatus = readSoilSensor(soilSensors[1].sensor, soilSensors[1].power, soil2);
      delay(200);
      int soil3SensorStatus = readSoilSensor(soilSensors[2].sensor, soilSensors[2].power, soil3);

      // calc overall sensor status
      currentSensorStatus = (bmeSensorStatus << 6) + (soil1SensorStatus << 4) +
        (soil2SensorStatus << 2) + soil3SensorStatus;

      if (currentSensorStatus >= 0) {
          lastMeasurement = 0;
          postToParticle();
          dumpSerial();
      }
    }

    // update cloud on timer or sensor data changes
    if (lastPost > POST_RATE || currentSensorStatus != lastSensorStatus) {
      while (postToPhant() <= 0) { // publish to phant
        Serial.println("Phant post failed. Trying again.");
				// Delay 1s, so we don't flood the server. Little delay's allow the Photon time
				// to communicate with the Cloud.
        for (int i=0; i<1000; i++) {
              delay(1);
        }
      }
      lastPost = 0;
    }

    lastSensorStatus = currentSensorStatus;
}

// post to Particle cloud
void postToParticle() {
  sprintf(publishString,"{\"status\": %d, \"temp\": %0.2f, \"pressure\": %0.2f, \"humidity\": %0.2f, \"soil1\": %u, \"soil2\": %u, \"soil3\": %u}",
    currentSensorStatus, temperature, pressure, humidity, soil1, soil2, soil3);
  Particle.publish("sensor",publishString);
}

// post to SparkFun phant cloud storage
int postToPhant() {
  phant.add("status", currentSensorStatus);
  phant.add("temp", temperature, 2);
  phant.add("pressure", pressure, 2);
  phant.add("humidity", humidity, 2);
  phant.add("soil1", soil1);
  phant.add("soil2", soil2);
  phant.add("soil3", soil3);
  return phant.particlePost();
}

// read BME280 sensor data
int readBMESensor() {
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
int readSoilSensor(const int soilSensor, const int soilSensorPower, int &soilValue) {
    digitalWrite(led, HIGH);
    digitalWrite(soilSensorPower, HIGH); //turn sensor power on
    delay(10); // give some time to settle
    soilValue = analogRead(soilSensor);
    digitalWrite(soilSensorPower, LOW); //turn sensor power off
    delay(190);
    digitalWrite(led, LOW);

    if (soilValue < soildThresholdLow) {
      return 1; // indicate soil is to low
    } else if (soilValue > soildThresholdHigh) {
      return 2; // indicate soil is to high
    } else {
      return 0; // indicate normal reading
    }
    return 0;
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

  Serial.print("Soil 2 = ");
  Serial.println(soil2);

  Serial.print("Soil 3 = ");
  Serial.println(soil3);
}
