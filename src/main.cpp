#include <main.h>

AxisAssembly altAssembly("ALT", 6, 5, 10, 3, A2, A0, 1782770,110.);
AxisAssembly azAssembly("AZ", 9, 4, 11, 2, A3, A1, 1782460,90.);

AsyncStream<50> serial(&Serial, '\n');

void setup()
{
    Serial.begin(115200);

    attachInterrupt(digitalPinToInterrupt(altAssembly.enc), altEncoderInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(azAssembly.enc), azEncoderInterrupt, CHANGE);

    altAssembly.stopMotion();
    azAssembly.stopMotion();
}


void loop()
{
    parsing();

    // Update and correct velocity
    static uint32_t tmr2 = millis();
    if (millis() - tmr2 > 5)
    {
        altAssembly.correctVelocity();
        azAssembly.correctVelocity();

        tmr2 = millis();
    }

    // Debug print
    static uint32_t tmr1 = millis();
    if (millis() - tmr1 > 400)
    {
        altAssembly.printInfo();
        azAssembly.printInfo();

        tmr1 = millis();
    }
}

void parsing()
{
    if (serial.available())
    {
        Parser data(serial.buf, ',');  // отдаём парсеру
        int32_t ints[10];           // массив для численных данных
        data.parseInts(ints);   // парсим в него


        switch (ints[0])
        {
            case 0:
                azAssembly.motorGo(ints[1], ints[2]);
                break;
            case 1:
                altAssembly.motorGo(ints[1], ints[2]);
                break;

                // Set target velocity mRad/s
            case 2:
                if (ints[1] == 0)
                {
                    azAssembly.setTargetVelocity(ints[2]);
                } else if (ints[1] == 1)
                {
                    Serial.println(ints[2]);
                    altAssembly.setTargetVelocity(ints[2]);
                }
                break;

                // Set PID parameters
            case 3:
                if (ints[1] == 0)
                {
                    azAssembly.setPIDParameters(ints[2], ints[3], ints[4]);
                } else if (ints[1] == 1)
                {
                    altAssembly.setPIDParameters(ints[2], ints[3], ints[4]);
                }

                break;

                // Start motion
            case 4:
                if (ints[1] == 0)
                {
                    azAssembly.startMotion();
                } else if (ints[1] == 1)
                {
                    altAssembly.startMotion();
                }
                break;

                // Stop motion
            case 5:
                if (ints[1] == 0)
                {
                    azAssembly.stopMotion();
                } else if (ints[1] == 1)
                {
                    altAssembly.stopMotion();
                }
                break;
        }
    }
}

void altEncoderInterrupt()
{
    altAssembly.encoderInterrupt();
}

void azEncoderInterrupt()
{
    azAssembly.encoderInterrupt();
}

