No código original não é utilizado loop, pois trabalha-se com o deep sleep do esp, então, basicamente o código é executado uma vez em cada momento que o dispositivo liga.

"waking from deep sleep means the contents of memory are
unknown, so the entire program is read back into memory from flash
and we execute the setup() routine"


Ordem de execução das funções

Toda lógica da aplicação acontece na função setup()

setup()
    -> setCpuFrequencyMhz(80);
    -> setupMeasurement();
        -> timerAttachInterrupt()
        -> timerAlarmWrite()
        -> timerAlarmEnable()
            ->onTimer()
    makeMeasurement()
        -> readAnalogSamples();
        if (quantidade de amostras lidas == a quantidade de amostras definida)
        -> measureRms()
        -> cvtRmsToAmps()
    