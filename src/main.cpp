#include <Arduino.h>

#define CLOCKRATE 80000000 /* Hz */
#define TIMERDIVIDER 4

#define OVER_SAMPLE_RATIO (16)
#define CYCLES (20)
#define NSAMPLES (OVER_SAMPLE_RATIO * CYCLES)

#define ADC_BITS 12
#define ADC_COUNTS (1 << ADC_BITS) // 4096

#define VOLTAGE_ADC_PIN (34)
#define CURRENT_ADC_PIN (35)

#define WANTSERIAL (1)

volatile int sampleCount = NSAMPLES;
volatile int voltageSamples[NSAMPLES];
volatile int currentSamples[NSAMPLES];

const uint8_t pinVoltageAdc = 34;
const uint8_t pinCurrentAdc = 35;

hw_timer_t *My_timer = NULL;

struct measurements {
  float Vrms;
  float Irms;
};

/**
 * Timer Interrupt Service Routine (ISR)
 * 
 * to read the ADC and store samples in a buffer.
 * Then in mainline code, we will do the RMS calculation after all the
 * samples are available.
 * placing a function into IRAM
*/
void IRAM_ATTR onTimer() {
  if ((sampleCount >= 0) && (sampleCount < NSAMPLES)) {
    Serial.println(analogRead(CURRENT_ADC_PIN));
    voltageSamples[sampleCount++] = analogRead(VOLTAGE_ADC_PIN);
    currentSamples[sampleCount++] = analogRead(CURRENT_ADC_PIN);
  }
}

void setupMeasurement() {
  My_timer = timerBegin(
    1, // Timer we want to use
    TIMERDIVIDER, // prescaler 80MHz/4 = 20 MHz tick rate 
    true // Reloads the counter to have more than 1 interruption
    ); 
  timerAttachInterrupt(My_timer, &onTimer, true); // when the timer interrupt happens, the function onTimer() will be called

  float measIntervalSec = 1.0 / ( 60.0 * OVER_SAMPLE_RATIO);

  int count = (int)(measIntervalSec*CLOCKRATE/TIMERDIVIDER + 0.5);

  timerAlarmWrite(My_timer, count, true); // timer for interrupts
  timerAlarmEnable(My_timer); // and finally, Enable the dang interrupt
}

void readAnalogSamples() {
  int waitDelay = 17 * CYCLES;
  sampleCount = 0; // triggers the ISR to start reading the samples

  delay(waitDelay);

  if (sampleCount != NSAMPLES) {
    Serial.print("ADC processing is not working.");
  }

  timerWrite(My_timer, 0); // disable timer, we're done with interrupts
}

struct measurements measureRms(int* voltageSamples, int* currentSamples, int nsamples) {
  struct measurements eletricMeasurements;
  int32_t sumVoltageSamples = 0;
  int32_t sumCurrentSamples = 0;

  for (int i = 0; i < nsamples; i++) {
    sumVoltageSamples += voltageSamples[i];
    sumCurrentSamples += currentSamples[i];
  }

  int voltageMean = (int)(sumVoltageSamples / (int32_t)(nsamples));
  int currentMean = (int)(sumCurrentSamples / (int32_t)(nsamples));

  int32_t sumVoltage = 0;
  int32_t sumCurrent = 0;
  for (int i = 0; i < nsamples; i++) {
    int32_t y_voltage = (voltageSamples[i] - voltageMean);
    int32_t y_current = (currentSamples[i] - currentMean);
  
    sumVoltage += y_voltage * y_voltage;
    sumCurrent += y_current * y_current;
  }

  float ym_voltage = (float) sumVoltage / (float) nsamples;
  float ym_current = (float) sumCurrent / (float) nsamples;

  float Vrms = sqrt(ym_voltage);
  float Irms = sqrt(ym_current);

  eletricMeasurements.Vrms = Vrms * 3.3 / 4096.0;
  eletricMeasurements.Irms = Irms * 3.3 / 4096.0;

  return eletricMeasurements;
}

struct measurements makeMeasurement() {
  struct measurements eletricMeasurements;

  readAnalogSamples();
  if (sampleCount == NSAMPLES) {
    eletricMeasurements = measureRms((int*) voltageSamples, (int*) currentSamples, NSAMPLES);
  }

  return eletricMeasurements;
}

void setup() {
  Serial.begin(115200);

  setupMeasurement();

  struct measurements teste;

  teste = makeMeasurement();
}

void loop() {
  
}
