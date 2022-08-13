//
// Created by Ivan Kalesnikau on 12.08.2022.
//

#ifndef ASTROMOUNTDRIVER_AXISASSEMBLY_H
#define ASTROMOUNTDRIVER_AXISASSEMBLY_H

#include <Arduino.h>

#define COUNTER_PERIOD 1000

class AxisAssembly
{
public:
    uint8_t keyA, keyB, pwm, enc, cp, ep;
    volatile int32_t encCounter = 0;
    volatile uint32_t encIncCounter;
    volatile uint32_t encTimer;
    uint32_t encVelocity;
    uint8_t dirMem, pwmMem;

private:

public:
    AxisAssembly(const int &keyA, const int &keyB, const int &pwm, const int &enc, const int &cp,
                 const int &ep);

    ~AxisAssembly();

    void encoderInterrupt();

    void motorOff();

    void motorGo(uint8_t dir, uint8_t pwm);

    int readCurrent();

    int readEnable();

private:

};


#endif //ASTROMOUNTDRIVER_AXISASSEMBLY_H
