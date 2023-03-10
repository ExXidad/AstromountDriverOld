//
// Created by Ivan Kalesnikau on 12.08.2022.
//

#ifndef ASTROMOUNTDRIVER_MAIN_H
#define ASTROMOUNTDRIVER_MAIN_H

#include <Arduino.h>
#include <../.pio/libdeps/nanoatmega328/AsyncTimer/src/AsyncTimer.h>
#include <Parser.h>
#include <AsyncStream.h>
#include <AxisAssembly.h>


void setup();

void loop();

void altEncoderInterrupt();

void azEncoderInterrupt();

void parsing();

void printRoutePoints();

#endif //ASTROMOUNTDRIVER_MAIN_H
