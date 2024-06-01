struct ElectricalMeasurements {
  double vrms;
  double irms;
  double realPower;
  double apparentPower;
  double powerFactor;
};

void setupMeasurement();
struct ElectricalMeasurements makeMeasurement();