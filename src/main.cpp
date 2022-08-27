#include <main.h>

#define NUMBEROFASSEMBLIES 2

AxisAssembly axisAssemblies[NUMBEROFASSEMBLIES] = {
        AxisAssembly("AZ", 9, 4, 11, 2, A3, A1, 1782460, 85.),
        AxisAssembly("ALT", 6, 5, 10, 3, A2, A0, 1782770, 110.)
};

AsyncStream<50> serial(&Serial, '\n');

#define MAXNUMBEROFPOINTS 100

double routePoints[NUMBEROFASSEMBLIES + 1][MAXNUMBEROFPOINTS] = {};
uint8_t numberOfPoints = 0;

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
    if (millis() - tmr1 > 200)
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

        // Read route array
        if (commandNumber == 6)
        {
            int N = data.getInt(1);
            if (N > MAXNUMBEROFPOINTS) return;
            numberOfPoints = N;
            int i = 0;
            while (i < N)
            {
                if (serial.available() > 0)
                {
                    Parser arrayData(serial.buf, ',');
                    arrayData.split();
                    routePoints[0][i] = arrayData.getFloat(0);
                    routePoints[1][i] = arrayData.getFloat(1);
                    routePoints[2][i] = arrayData.getFloat(2);
                    ++i;
                }
            }

            for (int j = 0; j < NUMBEROFASSEMBLIES; ++j)
            {
                axisAssemblies[j].addRoute(routePoints[0], routePoints[j + 1], numberOfPoints);
            }

            Serial.println("Done.");
//            printRoutePoints();
            return;
        }

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
                for (int i = 0; i < NUMBEROFASSEMBLIES; ++i)
                {
                    axisAssemblies[i].setPIDParameters(data.getFloat(2), data.getFloat(3), data.getFloat(4));
                }
                break;

                // Start motion
            case 4:
                for (int i = 0; i < NUMBEROFASSEMBLIES; ++i)
                {
                    axisAssemblies[i].startMotion();
                }
                break;

                // Stop motion
            case 5:
                for (int i = 0; i < NUMBEROFASSEMBLIES; ++i)
                {
                    axisAssemblies[i].stopMotion();
                }
                break;

                // Print Route array
            case 7:
                printRoutePoints();
                break;

                // Move to start position
            case 8:
                for (int i = 0; i < NUMBEROFASSEMBLIES; ++i)
                {
                    axisAssemblies[i].moveToRouteStartPosition();
                }
                break;

                // Start route
            case 9:
                for (int i = 0; i < NUMBEROFASSEMBLIES; ++i)
                {
                    axisAssemblies[i].startRoute();
                }
                break;

            default:
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

void printRoutePoints()
{
    for (int n = 0; n < numberOfPoints; ++n)
    {
        for (int i = 0; i < NUMBEROFASSEMBLIES + 1; ++i)
        {
            Serial.print(routePoints[i][n]);
            Serial.print("\t");
        }
        Serial.println();
    }
}