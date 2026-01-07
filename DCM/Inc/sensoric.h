#ifndef SENSORIC_H
#define SENSORIC_H

#include "can.h"

void sensoric_parse(uint16_t canID, volatile void *payload);