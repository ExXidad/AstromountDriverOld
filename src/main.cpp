#include <main.h>

AxisAssembly altAssembly(6, 5, 10, 3, A2, A0);
AxisAssembly azAssembly(9, 4, 11, 2, A3, A1);

AsyncStream<50> serial(&Serial, '\n');

void setup()
{
    Serial.begin(115200);

    attachInterrupt(digitalPinToInterrupt(altAssembly.enc), altEncoderInterupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(azAssembly.enc), azEncoderInterupt, CHANGE);
}

void loop()
{
    parsing();

    static uint32_t tmr1 = 0;

    if (millis() - tmr1 > 300)
    {
        Serial.print("#:ALT:");
        Serial.print(altAssembly.dirMem, 1);
        Serial.print(":");
        Serial.print(altAssembly.pwmMem, 3);
        Serial.print(":");
        Serial.print(altAssembly.encCounter, 10);
        Serial.print(":");
        Serial.print(altAssembly.encVelocity, 10);
        Serial.println(":#");

        Serial.print("#:AZ:");
        Serial.print(azAssembly.dirMem, 1);
        Serial.print(":");
        Serial.print(azAssembly.pwmMem, 3);
        Serial.print(":");
        Serial.print(azAssembly.encCounter, 10);
        Serial.print(":");
        Serial.print(azAssembly.encVelocity, 10);
        Serial.println(":#");
        tmr1 = millis();
    }
}

void parsing()
{
    if (serial.available())
    {
        Parser data(serial.buf, ',');  // отдаём парсеру
        int ints[10];           // массив для численных данных
        data.parseInts(ints);   // парсим в него

        switch (ints[0])
        {
            case 0:
                altAssembly.motorGo(ints[1], ints[2]);
                break;
            case 1:
                azAssembly.motorGo(ints[1], ints[2]);
                break;
        }
    }
}

void altEncoderInterupt()
{
    altAssembly.encoderInterrupt();
}

void azEncoderInterupt()
{
    azAssembly.encoderInterrupt();
}

