
#include <option.h>

#include <cstring>

using namespace std;

namespace NaoControl {
Option::Option(const char* n) : name(n), buffer(nullptr), length(0) {}

uint8_t* Option::save(uint8_t* start) {
    memcpy(start, buffer, length);
    return start + length;
}

} /* namespace NaoControl */
