char timeStamp[22];
char publishString[64];

void setup() {
    pinMode(D7, OUTPUT);
    uint32_t startTime = millis();
    while(millis() - startTime < 5000UL) {
        digitalWrite(D7, HIGH);
        Spark.process();
        delay(100);
        digitalWrite(D7, LOW);
        Spark.process();
        delay(100);
    }
}

void loop() {

    int wifiSignal = WiFi.RSSI();
    sprintf(publishString,"{\"Wifi\": \"%s\", \"Signal\": %d}",WiFi.SSID(),wifiSignal);
    Particle.publish("Wifi",publishString);

    time_t time = Time.now();
    Time.format(time, TIME_FORMAT_ISO8601_FULL).toCharArray(timeStamp, 64);
    Particle.publish("TimeStamp", timeStamp);

    digitalWrite(D7, HIGH);
    delay(1000);
    digitalWrite(D7, LOW);

    System.sleep(SLEEP_MODE_DEEP, 60);
}
