#define INT_SAVE(var)                   \
    memcpy(ptr, &(var), sizeof((var))); \
    ptr += sizeof((var))

#include <intoption.h>
#include <visstring.h>

#include <cstring>
#include <utility>

namespace NaoControl {

template <typename T>
IntOption<T>::IntOption(const char* cname, T* val, T min, T max, T step, std::function<void(T)> callb)
    : Option(cname), value(val), getCallback(nullptr), setCallback(std::move(callb)) {

    VisString optionName(cname);
    length = sizeof(uint8_t) /* type */ + optionName.size() + sizeof(int8_t) /* type size */ + sizeof(min) +
             sizeof(max) + sizeof(step) + sizeof(*val);

    int8_t intSize = sizeof(*val);

    buffer = (uint8_t*)malloc(length);
    uint8_t* ptr = buffer;
    *ptr = (uint8_t)O_INT;
    ptr++;
    ptr = optionName.save(ptr);
    INT_SAVE(intSize);
    INT_SAVE(min);
    INT_SAVE(max);
    INT_SAVE(step);
}

template <typename T>
IntOption<T>::IntOption(const char* cname, T min, T max, T step, std::function<T(void)> getCallback,
                        std::function<void(T)> setCallback)
    : Option(cname),
      value(nullptr),
      getCallback(std::move(std::move(getCallback))),
      setCallback(std::move(setCallback)) {

    VisString optionName(cname);
    length = sizeof(uint8_t) /* type */ + optionName.size() + sizeof(int8_t) /* type size */ + sizeof(min) +
             sizeof(max) + sizeof(step) + sizeof(T);

    int8_t intSize = sizeof(T);

    buffer = (uint8_t*)malloc(length);
    uint8_t* ptr = buffer;
    *ptr = (uint8_t)O_INT;
    ptr++;
    ptr = optionName.save(ptr);
    INT_SAVE(intSize);
    INT_SAVE(min);
    INT_SAVE(max);
    INT_SAVE(step);
}

template <typename T>
uint8_t* IntOption<T>::save(uint8_t* start) {
    uint8_t* ptr = buffer;
    ptr += length - sizeof(*value);

    T tmpVal = 0;

    if (value != nullptr) {
        tmpVal = *value;
    } else {
        tmpVal = getCallback();
    }

    INT_SAVE(tmpVal);

    return Option::save(start);
}

template <typename T>
void IntOption<T>::setValue(const uint8_t* buffer, int len) {
    T newValue;

    if (len != sizeof(newValue))
        return;

    memcpy(&newValue, buffer, len);

    if (value != nullptr)
        *value = newValue;

    if (setCallback != nullptr) {
        setCallback(newValue);
    }
}

template class IntOption<int>;
template class IntOption<int16_t>;
template class IntOption<unsigned char>;
template class IntOption<signed char>;
}  // namespace NaoControl
