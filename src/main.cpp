#include <main.h>

#define NUMBEROFASSEMBLIES 2

AxisAssembly axisAssemblies[NUMBEROFASSEMBLIES] = {
        AxisAssembly("AZ", 9, 4, 11, 2, A3, A1, 1782460, 85.),
        AxisAssembly("ALT", 6, 5, 10, 3, A2, A0, 1782770, 110.)
};

AsyncStream<50> serial(&Serial, '\n');

void setup()
{
    Serial.begin(115200);

    attachInterrupt(digitalPinToInterrupt(axisAssemblies[0].enc), azEncoderInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(axisAssemblies[1].enc), altEncoderInterrupt, CHANGE);

    for (int i = 0; i < NUMBEROFASSEMBLIES; ++i)
    {
        axisAssemblies[i].stopMotion();
    }
}


void loop()
{
    parsing();

    // Update and correct position
    static uint32_t tmr2 = millis();
    if (millis() - tmr2 > 5)
    {
        for (int i = 0; i < NUMBEROFASSEMBLIES; ++i)
        {
            axisAssemblies[i].correctPosition();
        }

        tmr2 = millis();
    }

    // Debug print
    static uint32_t tmr1 = millis();
    if (millis() - tmr1 > 400)
    {
        for (int i = 0; i < NUMBEROFASSEMBLIES; ++i)
        {
            axisAssemblies[i].printInfo();
        }

        tmr1 = millis();
    }
}

void parsing()
{
    if (serial.available())
    {
        Parser data(serial.buf, ',');  // отдаём парсеру
        data.split();

        uint8_t commandNumber = data.getInt(0);
        uint8_t assemblyNumber = data.getInt(1);


        switch (commandNumber)
        {
            // Simple motion
            case 0:
                axisAssemblies[assemblyNumber].motorGo(data.getInt(2), data.getInt(3));
                break;

                // Set zero position

            case 1:
                for (int i = 0; i < NUMBEROFASSEMBLIES; ++i)
                {
                    axisAssemblies[i].setZeroPosition();
                }

                // Set target position rad
            case 2:
                axisAssemblies[assemblyNumber].setTargetPosition(data.getFloat(2));
                break;

                // Set PID parameters
            case 3:
                axisAssemblies[assemblyNumber].setPIDParameters(data.getFloat(2), data.getFloat(3), data.getFloat(4));
                break;

                // Start motion
            case 4:
                axisAssemblies[assemblyNumber].startMotion();
                break;

                // Stop motion
            case 5:
                axisAssemblies[assemblyNumber].stopMotion();
                break;


        }
    }
}

void azEncoderInterrupt()
{
    axisAssemblies[0].encoderInterrupt();
}

void altEncoderInterrupt()
{
    axisAssemblies[1].encoderInterrupt();
}

