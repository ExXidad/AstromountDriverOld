//
// Created by Ivan Kalesnikau on 12.08.2022.
//

#ifndef ASTROMOUNTDRIVER_AXISASSEMBLY_H
#define ASTROMOUNTDRIVER_AXISASSEMBLY_H

#include <Arduino.h>

class AxisAssembly
{
public:

    String name;
    uint8_t keyA, keyB, pwm, enc, cp, ep;
    volatile int32_t encCounter = 0;
    volatile uint32_t encTimer;
    double encVelocity;
    uint8_t dirMem, pwmMem;
    int32_t encTmpCounter;
    double currentVelocity;

    double countsPerRevolution;
    double countsPerRadian;

    // Counts per us
    double targetVelocity = 0;

    // PID parameters
    double p, i, d;
    double kp = 0, ki = 0, kd = 0;
    int integrationLength = 100;
    double C, pidPWM;
    double prevError;
    uint32_t prevCallTime;

    // Flag
    bool moving = false;


private:

public:
    AxisAssembly(const String &name, const int &keyA, const int &keyB, const int &pwm, const int &enc, const int &cp,
                 const int &ep, const uint32_t &countsPerRevolution, const double &maxAchievedVelMRPS);

    ~AxisAssembly();

    void encoderInterrupt();

    void motorOff();

    void motorGo(uint8_t dir, uint8_t pwm);

    int readCurrent();

    int readEnable();

    void updateVelocity();

    double slewAtConstantVelocity();

    void correctVelocity();

    void setTargetVelocity(const double &uRadPerSecond);

    void setPIDParameters(const double &kp, const double &ki, const double &kd);

    void startMotion();

    void stopMotion();

    double CPUSfromMRPS(const double &mRadPerSecond);

    double CPUStoMRPS(const double &vel);

    double countsToRad(const double &counts);

    double radToCounts(const double &rad);

    void printInfo();

    void resetPID();

private:

};


#endif //ASTROMOUNTDRIVER_AXISASSEMBLY_H
