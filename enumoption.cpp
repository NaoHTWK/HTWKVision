#include "enumoption.h"

#define SAVE(var)                       \
    memcpy(ptr, &(var), sizeof((var))); \
    ptr += sizeof((var))

#include <visstring.h>

#include <cstring>

namespace NaoControl {
EnumItem::EnumItem(const char* name, int value) {
    VisString vName(name);
    len = vName.size() + sizeof(value);

    buffer = (uint8_t*)malloc(len);
    uint8_t* ptr = buffer;
    ptr = vName.save(ptr);
    SAVE(value);
}

uint8_t* EnumItem::save(uint8_t* ptr) {
    memcpy(ptr, buffer, len);
    return ptr + len;
}

EnumOption::EnumOption(const char* cname, int* val, std::function<void(int)> callb)
    : Option(cname), value(val), callback(std::move(callb)) {
    VisString optionName(cname);
    length = sizeof(uint8_t) /* type */ + optionName.size() + sizeof(*val) + sizeof(int32_t) /* item count */;
}

uint8_t* EnumOption::save(uint8_t* start) {
    VisString optionName(name.c_str());
    uint8_t* ptr = start;
    *ptr = (uint8_t)O_ENUM;
    ptr++;
    ptr = optionName.save(ptr);
    SAVE(*value);

    int itemCount = enumItems.size();
    SAVE(itemCount);

    for (auto& enumItem : enumItems) {
        ptr = enumItem->save(ptr);
    }

    return ptr;
}

void EnumOption::setValue(const uint8_t* buffer, int len) {
    int newValue;

    if (len != sizeof(newValue))
        return;

    memcpy(&newValue, buffer, len);
    *value = newValue;

    if (callback != nullptr) {
        callback(*value);
    }
}

void EnumOption::addEnumItem(EnumItem* item) {
    enumItems.emplace_back(item);
    length += item->size();
}
} /* namespace NaoControl */
