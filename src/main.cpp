#include <Arduino.h>
#include <EnergyMeter.h>

void setup() {
  Serial.begin(115200);

  setupMeasurement();

  struct ElectricalMeasurements eletricMeasurements;

  eletricMeasurements = makeMeasurement();

  Serial.print("Vrms: ");
  Serial.print(eletricMeasurements.vrms, 5);
  Serial.print(" Irms: ");
  Serial.print(eletricMeasurements.irms, 5);
  Serial.print(" Real Power: ");
  Serial.print(eletricMeasurements.realPower, 5);
  Serial.print(" Apparent Power: ");
  Serial.print(eletricMeasurements.apparentPower, 5);
  Serial.print(" Power factor: ");
  Serial.println(eletricMeasurements.powerFactor, 5);
}

void loop() {
  
}
