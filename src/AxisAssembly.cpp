//
// Created by Ivan Kalesnikau on 12.08.2022.
//

#include "AxisAssembly.h"

AxisAssembly::AxisAssembly(const int &keyA, const int &keyB, const int &pwm, const int &enc, const int &cp,
                           const int &ep)
{
    this->keyA = keyA;
    this->keyB = keyB;
    this->pwm = pwm;
    this->enc = enc;
    this->cp = cp;
    this->ep = ep;

    pinMode(keyA, OUTPUT);
    pinMode(keyB, OUTPUT);
    pinMode(pwm, OUTPUT);
    pinMode(enc, INPUT);

    motorOff();
    encTimer = micros();
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

    ++encIncCounter;

    if (encIncCounter >= COUNTER_PERIOD)
    {
        encVelocity = micros() - encTimer;
        encTimer = micros();
        encIncCounter = 0;
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
        Serial.println(dirMem);
        Serial.println(pwmMem);
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
