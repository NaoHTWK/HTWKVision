/*
 * booloption.cpp
 *
 *  Created on: May 20, 2013
 *      Author: tkalbitz
 */

#include "booloption.h"

#define SAVE(var)                       \
    memcpy(ptr, &(var), sizeof((var))); \
    ptr += sizeof((var))

#include <visstring.h>

#include <cstring>

namespace NaoControl {
BoolOption::BoolOption(const char* cname, bool* val, std::function<void(bool)> callb)
    : Option(cname), value(val), callback(std::move(callb)) {

    VisString optionName(cname);
    length = sizeof(uint8_t) /* type */ + optionName.size() + sizeof(*val);

    buffer = (uint8_t*)malloc(length);
    uint8_t* ptr = buffer;
    *ptr = (uint8_t)O_BOOL;
    ptr++;
    optionName.save(ptr);
}

uint8_t* BoolOption::save(uint8_t* start) {
    uint8_t* ptr = buffer;
    ptr += length - sizeof(*value);
    SAVE(*value);

    return Option::save(start);
}

void BoolOption::setValue(const uint8_t* buffer, int len) {
    bool newValue;

    if (len != sizeof(newValue))
        return;

    memcpy(&newValue, buffer, len);
    *value = newValue;

    if (callback != nullptr) {
        callback(*value);
    }
}

} /* namespace NaoControl */
