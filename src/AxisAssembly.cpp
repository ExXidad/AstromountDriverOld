//
// Created by Ivan Kalesnikau on 12.08.2022.
//

#include "AxisAssembly.h"

AxisAssembly::AxisAssembly(const String &name, const int &keyA, const int &keyB, const int &pwm, const int &enc,
                           const int &cp,
                           const int &ep, const uint32_t &countsPerRevolution,
                           const double &maxAchievedVelMRPS)
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
    C = 0.95 * 127. / CPUSfromMRPS(maxAchievedVelMRPS);
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

int AxisAssembly::readCurrent()
{
    return analogRead(cp);
}

int AxisAssembly::readEnable()
{
    return analogRead(ep);
}

void AxisAssembly::updateVelocity()
{
    int32_t encCounterDifference = encCounter - encTmpCounter;
    uint32_t encTimerDifference = micros() - encTimer;

    encVelocity = 1. * encCounterDifference / encTimerDifference;

    encTimer = micros();
    encTmpCounter = encCounter;

    currentVelocity = encVelocity;
}

double AxisAssembly::slewAtConstantVelocity()
{
    return 0;
}

void AxisAssembly::correctVelocity()
{
    updateVelocity();
    if (moving)
    {
        double error = targetVelocity - currentVelocity;
        uint32_t dt = micros() - prevCallTime;

        p = error;
        i = i + error * dt;
        d = (error - prevError) / dt;

        pidPWM += C * kp * p;// + ki * i + kd * d;
        int absOutput = abs(pidPWM);
        motorGo(pidPWM > 0 ? 1 : 2, absOutput > 127 ? 127 : absOutput);

        prevError = error;
        prevCallTime = micros();
    }
}

void AxisAssembly::setTargetVelocity(const double &uRadPerSecond)
{
    targetVelocity = CPUSfromMRPS(uRadPerSecond);
    pidPWM = C * targetVelocity;

    resetPID();
}

void AxisAssembly::setPIDParameters(const double &kp, const double &ki, const double &kd)
{
    this->kp = kp / 1000.;
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
    motorOff();
    resetPID();
}

double AxisAssembly::CPUSfromMRPS(const double &mRadPerSecond)
{
    return mRadPerSecond * 0.000000001 * countsPerRadian;
}

double AxisAssembly::CPUStoMRPS(const double &vel)
{
    return vel * 1000000000. / countsPerRadian;
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
    Serial.print(countsToRad(encCounter), 3);
    Serial.print(",");
    Serial.print(CPUStoMRPS(currentVelocity), 10);
    Serial.print(",");
    Serial.print(CPUStoMRPS(targetVelocity), 10);
    Serial.print(",");
    Serial.print(CPUSfromMRPS(50) * C, 4);
    Serial.print(",");
    Serial.print(CPUStoMRPS(kp * C), 4);
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
