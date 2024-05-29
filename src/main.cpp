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
 * There's a buffer in memory for voltage and current (voltageSamples, currentSamples)
 * to store the max number of samples we're collecting (NSAMPLES = 320).
 * At each interruption, a sample is taken, so every 1041.6666 us.
 * 
 * The result of the samples is a 12 bit number from 0 to 4095. Stored in the 
 * next space avaible in the samples variables of voltage and current.
 * 
 * When the buffer is filled sampleCount >= NSAMPLES (sampleCount >= 320)
 * the ISR doesn't take any additional samples.
 * 
 * So after a bunch of timer interrupts, we ended up with the voltages and current
 * buffer's full of digitized measurements.
 * Once that is completely filled, we can proceed (in the main code, not inside the ISR)
 * to make the computations necessary to compute the RMS values. 
 * 
*/
void IRAM_ATTR onTimer() {
  if ((sampleCount >= 0) && (sampleCount < NSAMPLES)) {
    voltageSamples[sampleCount++] = analogRead(VOLTAGE_ADC_PIN);
    currentSamples[sampleCount++] = analogRead(CURRENT_ADC_PIN);
  }
}


/**
 * We are sampling 16 times per cycle
 * 60 Hz: this means there are 60 complete cycles per second
 * each cycle takes 0.016667 seconds per cycle
 * Since we want to sample 16 times within each cycle,
 * we need to take an ADC sample at 16.667/16 = 1041.667 us
 * 
*/
void setupMeasurement() {
  // Explanation prescaler (divider value to the clock frequency i.e. 80MHz.): 
  // We have a 16bit prescaler so we can set any value from 2 to 65536
  // Here we used 4, so the value after dividing will be 80MHz/4= 20MHz
  // This means in one second the timer will count from 0 to 20000000
  // For each count, the timer will take 0.05 (1/20M) (microsecond)
  My_timer = timerBegin(
    1, // Timer we want to use
    TIMERDIVIDER, // prescaler 80MHz/4 = 20 MHz tick rate
    true // Reloads the counter to have more than 1 interruption
  );

  // when the timer interrupt happens, the function onTimer() will be called
  timerAttachInterrupt(My_timer, &onTimer, true);

  // We measure at a rate to get exactly 16 samples for every sine wave
  // for 16x osr: 1041.67 us
  float measureRatePerInterval = 1.0 / ( 60.0 * OVER_SAMPLE_RATIO);

  // Calculates the amount of time between interrupts ~ 20833
  int amountTimeBetweenInterruption = (int)( measureRatePerInterval * CLOCKRATE / TIMERDIVIDER + 0.5);

  // Used for defining the value for which the timer will generate the interrupt
  // Regarding the second argument, remember that we set the prescaler in order for
  // this to mean the number of microseconds after which the interrupt should occur.
  // Clock frequency: 20 Mhz
  //
  // Important: We can use different prescaler values and in that case we need 
  // to do the calculations to know when the counter will reach a certain value.
  // 
  // We will each interruption in amountTimeBetweenInterruption (20833) / actual clock frequency (20M)
  timerAlarmWrite(My_timer, amountTimeBetweenInterruption, true);

  // Enable Timer with interrupt (Alarm Enable)
  timerAlarmEnable(My_timer);
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
