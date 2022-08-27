//
// Created by Ivan Kalesnikau on 12.08.2022.
//

#include "AxisAssembly.h"

AxisAssembly::AxisAssembly(const String &name, const int &keyA, const int &keyB, const int &pwm, const int &enc,
                           const int &cp,
                           const int &ep, const uint32_t &countsPerRevolution, const double &maxAchievedVelMRPS)
{
    this->name = name;
    this->keyA = keyA;
    this->keyB = keyB;
    this->pwm = pwm;
    this->enc = enc;
    this->cp = cp;
    this->ep = ep;
    this->countsPerRevolution = countsPerRevolution;

    countsPerRadian = this->countsPerRevolution / 6.28319;
    C = 0.95 * 127. / RPStoCPUS(maxAchievedVelMRPS);
    pidPWM = 0;

    pinMode(keyA, OUTPUT);
    pinMode(keyB, OUTPUT);
    pinMode(pwm, OUTPUT);
    pinMode(enc, INPUT);

    motorOff();
    updateVelocity();
}

AxisAssembly::~AxisAssembly()
= default;

void AxisAssembly::encoderInterrupt()
{
    switch (dirMem)
    {
        case 0:
            break;
        case 1:
            ++encCounter;
            break;
        case 2:
            --encCounter;
            break;
    }
}

// Отключение мотора
void AxisAssembly::motorOff()
{
    for (int i = 0; i < 2; i++)
    {
        digitalWrite(keyA, LOW);
        digitalWrite(keyB, LOW);
    }
    analogWrite(pwm, 0);
    pwmMem = 0;
    dirMem = 0;
}

void AxisAssembly::motorGo(uint8_t dirToGo, uint8_t pwmToWrite)
{
// если направление совпадает со значениями направлений:
// 0 -- стоять
    if (dirToGo <= 3)
    {
// если направление мотора по часовой или плавный стоп,
// устанавливаем соответствующие значения ключа А выбранного мотора:
        if (dirToGo <= 1)
            digitalWrite(keyA, HIGH);
        else
            digitalWrite(keyA, LOW);

// если направление мотора по часовой или резкий стоп,
// устанавливаем соответствующие значения ключа B выбранного мотора:
        if ((dirToGo == 0) || (dirToGo == 2))
            digitalWrite(keyB, HIGH);
        else
            digitalWrite(keyB, LOW);
// устанавливаем ШИМ выбранного мотора
        analogWrite(pwm, pwmToWrite);

        this->dirMem = dirToGo;
        this->pwmMem = pwmToWrite;
    }
}

void AxisAssembly::updateVelocity()
{
    double posDifference = countsToRad(encCounter - prevEncCounter);
    double timeDifference = (micros() - encTimer) * 0.000001;

    encVelocity = posDifference / timeDifference;

    encTimer = micros();

    prevEncCounter = encCounter;
    currentVelocity = encVelocity;
}

void AxisAssembly::setPIDParameters(const double &kp, const double &ki, const double &kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void AxisAssembly::startMotion()
{
    moving = true;
    resetPID();
}

void AxisAssembly::stopMotion()
{
    moving = false;
    movingByRoute = false;
    motorOff();
    resetPID();
}

double AxisAssembly::RPStoCPUS(const double &radPerSecond)
{
    return radPerSecond * 0.000000001 * countsPerRadian;
}

double AxisAssembly::CPUStoRPS(const double &countsPerUSecond)
{
    return countsPerUSecond * 1000000000. / countsPerRadian;
}

double AxisAssembly::countsToRad(const double &counts)
{
    return counts / countsPerRadian;
}

double AxisAssembly::radToCounts(const double &rad)
{
    return countsPerRadian * rad;
}

void AxisAssembly::printInfo()
{
    Serial.print(name);
    Serial.print(",");
    Serial.print(dirMem);
    Serial.print(",");
    Serial.print(pwmMem);
    Serial.print(",");
    Serial.print(countsToRad(encCounter), 10);
    Serial.print(",");
    Serial.print(targetPosition, 10);
    Serial.print(",");
    Serial.print(p, 4);
    Serial.print(",");
    Serial.print(kp, 4);
    Serial.print(",");
    Serial.print(pidPWM, 4);
    Serial.println(";");
}

void AxisAssembly::resetPID()
{
    i = 0;
    prevError = 0;
    prevCallTime = micros();
}

void AxisAssembly::setTargetPosition(const double &targetPosition)
{
    this->targetPosition = targetPosition;
}

void AxisAssembly::correctPosition()
{
//    updateVelocity();
    if (moving)
    {
        // Update target position
        if (movingByRoute)
        {
            double timeSinceMotionStarted = (micros() - startTime) * 0.000001;

            if (timeSinceMotionStarted < timePoints[0])
            {
                setTargetPosition(posPoints[0]);
            } else if (timeSinceMotionStarted > timePoints[numberOfPoints - 1])
            {
                setTargetPosition(posPoints[numberOfPoints - 1]);
            } else if (intervalIdx < numberOfPoints - 1)
            {
                if (timeSinceMotionStarted > timePoints[intervalIdx] &&
                    timeSinceMotionStarted < timePoints[intervalIdx + 1])
                {
                    setTargetPosition(posPoints[intervalIdx] +
                                      (posPoints[intervalIdx + 1] - posPoints[intervalIdx]) /
                                      (timePoints[intervalIdx + 1] - timePoints[intervalIdx]) *
                                      (timeSinceMotionStarted - timePoints[intervalIdx]));
                } else ++intervalIdx;
            }
        }

        // PID

        double error = targetPosition - countsToRad(encCounter);
        double dt = (micros() - prevCallTime) * 0.000001;

        p = error;
        i = i + error * dt;
        d = 1. * (error - prevError) / dt;

        pidPWM = kp * p + ki * i + kd * d;
        int absOutput = abs(pidPWM);
        motorGo(pidPWM > 0 ? 1 : 2, absOutput > 127 ? 127 : absOutput);

        prevError = error;
        prevCallTime = micros();
    }
}

void AxisAssembly::setZeroPosition()
{
    encCounter = 0;
}

void AxisAssembly::addRoute(double *timePoints, double *posPoints, const uint8_t &numberOfPoints)
{
    if (timePoints != nullptr && posPoints != nullptr && numberOfPoints > 0)
    {
        this->timePoints = timePoints;
        this->posPoints = posPoints;
        this->numberOfPoints = numberOfPoints;
        routeWasDefined = true;
    }
}

void AxisAssembly::startRoute()
{
    if (routeWasDefined)
    {
        movingByRoute = true;
        startMotion();
        intervalIdx = 0;
        startTime = micros();
    }
}

void AxisAssembly::moveToRouteStartPosition()
{
    if (routeWasDefined)
    {
        startMotion();
        setTargetPosition(posPoints[0]);
    }
}
