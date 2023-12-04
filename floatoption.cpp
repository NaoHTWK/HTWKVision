#include "floatoption.h"

#define SAVE(var)                       \
    memcpy(ptr, &(var), sizeof((var))); \
    ptr += sizeof((var))

#include <visstring.h>

#include <cstring>

namespace NaoControl {

FloatOption::FloatOption(const char* cname, float* val, float min, float max, float step,
                         std::function<void(float)> callb)
    : Option(cname), value(val), callback(std::move(callb)) {

    VisString optionName(cname);
    length = sizeof(uint8_t) /* type */ + optionName.size() + sizeof(min) + sizeof(max) + sizeof(step) + sizeof(*val);

    buffer = (uint8_t*)malloc(length);
    uint8_t* ptr = buffer;
    *ptr = (uint8_t)O_FLOAT;
    ptr++;
    ptr = optionName.save(ptr);
    SAVE(min);
    SAVE(max);
    SAVE(step);
}

uint8_t* FloatOption::save(uint8_t* start) {
    uint8_t* ptr = buffer;
    ptr += length - sizeof(*value);
    SAVE(*value);

    return Option::save(start);
}

void FloatOption::setValue(const uint8_t* buffer, int len) {
    float newValue;

    if (len != sizeof(newValue))
        return;

    memcpy(&newValue, buffer, len);
    *value = newValue;

    if (callback != nullptr) {
        callback(*value);
    }
}
}  // namespace NaoControl
