#include <SPI.h>
#include <LoRa.h>
#include <MPU9250.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;
MPU9250 mpu;
#define MOSFET_PIN 32

int counter = 0;
float lastAlts[3] = {-1000, -1000, -1000};
float first_alt = -1000;
bool ignite_watch = false;
int MIN_HEIGHT = 50;
int CUR_PRESSURE = 1010;

void setup() {
  pinMode(MOSFET_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  

  Serial.begin(9600);
  while (!Serial);
  Serial.println("LoRa Sender");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  
}

void ignite() {
  digitalWrite(MOSFET_PIN, HIGH);  
  delay(400);
  digitalWrite(MOSFET_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);
}

void addToArr(float new_val) {
  lastAlts[0] = lastAlts[1];
  lastAlts[1] = lastAlts[2];
  lastAlts[2] = new_val;
}

float avgAlt() {
  if (lastAlts[0] != -1000 && lastAlts[1] != -1000 && lastAlts[0] != -1000){
    return (lastAlts[0] + lastAlts[1] + lastAlts[2])/3;
  }
  return -1000;
}

void igniteWatcher(float new_alt){
  if (first_alt == -1000)
  {
    if (new_alt != -1000) {
      first_alt = new_alt;
    }
  }
  else {
    if (new_alt - first_alt > MIN_HEIGHT) {
      ignite_watch = true;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    if (ignite_watch){
      if (lastAlts[2] < lastAlts[1] && lastAlts[1] < lastAlts[0])
        ignite();
    }
  }
}

void loop() {
  float alt = bmp.readAltitude(CUR_PRESSURE);
  addToArr(alt);
  float avg_alt = avgAlt(); 
  LoRa.beginPacket();
  LoRa.print(alt);
  LoRa.print(';');
  LoRa.print(avg_alt);
  LoRa.print(';');
  LoRa.print(ignite_watch);
  LoRa.endPacket();
  igniteWatcher(avg_alt);
  counter++;

  delay(1000);
  // if (counter % 5 == 0) {
  //   ignite();
  // }
  
}
