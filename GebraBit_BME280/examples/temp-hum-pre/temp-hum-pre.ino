#include "GebraBit_BME280.h"

#define SPI_CS_Pin 10  // Change this if your CS pin is different

GebraBit_BME280 BME280;

void setup() {
  Serial.begin(9600);

  SPI.begin();
  pinMode(SPI_CS_Pin, OUTPUT);
  digitalWrite(SPI_CS_Pin, HIGH);

  GB_BME280_initialize(&BME280);
  
  // Check if the device ID matches the expected value
  GB_BME280_Get_Device_ID(&BME280);
  if (BME280.DEVICE_ID != 0x60) {
    Serial.println("Failed to initialize BME280 sensor!");
    while (1);
  } else {
    Serial.println("BME280 sensor initialized successfully.");
  }

  GB_BME280_Configuration(&BME280);
}

void loop() {

  GB_BME280_Get_Data(&BME280);

  Serial.print("Temperature: ");
  Serial.print(BME280.COMPENSATED_TEMPERATURE);
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(BME280.COMPENSATED_PRESSURE);
  Serial.println(" hPa");

  Serial.print("Humidity: ");
  Serial.print(BME280.COMPENSATED_HUMIDITY);
  Serial.println(" %");

  Serial.print("Altitude: ");
  Serial.print(BME280.ALTITUDE);
  Serial.println(" m");

  delay(1000);
}
