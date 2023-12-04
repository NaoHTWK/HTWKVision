#include "visstring.h"

#include <cstring>

#include <sockethelper.h>

namespace NaoControl
{
VisString::VisString(const char *str) {
    int sLen = strlen(str);
    len = sLen + sizeof(len);
    buffer = (uint8_t*)malloc(len);

    uint8_t* start = buffer;
    memcpy(start, &sLen, sizeof(sLen)); start += sizeof(sLen);
    memcpy(start, str, sLen);
}

VisString::~VisString() {
    free(buffer);
}

uint8_t* VisString::save(uint8_t* start) {
    memcpy(start, buffer, len);
    return start += len;
}

char* VisString::read(int sock) {
    int32_t len;
    if(!socketReadBytes(sock, sizeof(len), &len))
        return nullptr;

    char* string = (char*)calloc(1, len + 1);
    if(string == nullptr)
        return nullptr;

    socketReadBytes(sock, len, string);
    return string;
}


} /* namespace NaoControl */
