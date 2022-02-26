#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#define SEALEVELPRESSURE_HPA (1013.25)
float Altitude_Set;
Adafruit_BME680 bme; // I2C

#include <SD.h>
File myFile;

#include <Arduino.h>
#include <TM1637Display.h>
#define CLK 2
#define DIO 3
TM1637Display display(CLK, DIO);

void setup() {
  Serial.begin(9600);

  // BME680 고도센서 작동 상태 확인 ==================================
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while(1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  Serial.print("Sensor Setting... ");
  Altitude_Set = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print(Altitude_Set);
  Serial.println(" Setting done.");

  // SD카드 모듈 상태 확인
  Serial.print("Initializing SD card..."); 
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while(1);
  }
  Serial.println("initialization done.");

  // FND모듈 초기 설정
  display.setBrightness(16);//밝기 설정 범위는 0 ~ 16
}

void loop() {
  if (! bme.performReading()) { // 고도센서가 작동하지 않을 경우
    Serial.println("Failed to perform reading :(");
    return;
  }
  
  uint8_t data[] = { 0xff, 0xff, 0xff, 0xff };

  myFile = SD.open("sensor.txt", FILE_WRITE);
  if (myFile) { // SD카드가 삽입되어 있다면
    Serial.print(bme.temperature);
    Serial.print(" *C  ");
    myFile.print(bme.temperature);
    myFile.print(" ");
  
    Serial.print(bme.pressure / 100.0);
    Serial.print(" hPa  ");
    myFile.print(bme.pressure / 100.0);
    myFile.print(" ");
  
    Serial.print(bme.humidity);
    Serial.print(" %  ");
    myFile.print(bme.humidity);
    myFile.print(" ");
  
    Serial.print(bme.gas_resistance / 1000.0);
    Serial.print(" KOhms  ");
    myFile.print(bme.gas_resistance / 1000.0);
    myFile.print(" ");

    float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA) - Altitude_Set;
    Serial.print(altitude);
    Serial.println(" m");
    myFile.println(altitude);
    if(altitude >= 10000.0) display.showNumberDec(int(altitude) / 1000, false, 4, 0);
    else display.showNumberDec(int(altitude), false, 4, 0);
    
    myFile.close();
    //myFile.close();
    //Serial.println("done.");
  }
  delay(1000);
}
