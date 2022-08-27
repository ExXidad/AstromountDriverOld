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
    int32_t prevEncCounter;
    double currentVelocity = 0;

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

    double *timePoints = nullptr, *posPoints = nullptr;
    uint32_t startTime = 0;
    uint8_t numberOfPoints,intervalIdx = 0;

    // Another PID
    double targetPosition;

    // Flag
    bool moving = false;
    bool movingByRoute = false;
    bool routeWasDefined = false;


private:

public:
    AxisAssembly(const String &name, const int &keyA, const int &keyB, const int &pwm, const int &enc, const int &cp,
                 const int &ep, const uint32_t &countsPerRevolution, const double &maxAchievedVelMRPS);

    ~AxisAssembly();

    void encoderInterrupt();

    void motorOff();

    void motorGo(uint8_t dir, uint8_t pwm);

    void updateVelocity();

    void correctPosition();

    void setTargetPosition(const double &targetPosition);

    void setPIDParameters(const double &kp, const double &ki, const double &kd);

    void startMotion();

    void stopMotion();

    double RPStoCPUS(const double &radPerSecond);

    double CPUStoRPS(const double &countsPerUSecond);

    double countsToRad(const double &counts);

    double radToCounts(const double &rad);

    void printInfo();

    void resetPID();

    void setZeroPosition();

    void addRoute(double *timePoints, double *posPoints, const uint8_t &numberOfPoints);

    void startRoute();

    void moveToRouteStartPosition();

private:

};


#endif //ASTROMOUNTDRIVER_AXISASSEMBLY_H
