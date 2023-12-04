#include "optionset.h"

#include <visstring.h>

#include <cstring>

#define SAVE(var)                       \
    memcpy(ptr, &(var), sizeof((var))); \
    ptr += sizeof((var))

namespace NaoControl {
using namespace std;
OptionSet::OptionSet(const char* cname) : name(cname) {
    VisString vis(cname);
    length = sizeof(int32_t) /* my length */ + vis.size() + sizeof(uint32_t) /* option count */;
}

void OptionSet::addOption(Option* option) {
    OptionPtr opt(option);
    options[option->getName()] = opt;
    optionList.push_back(opt);
    length += option->size();
}

uint8_t* OptionSet::save(uint8_t* start) {
    VisString vis(name.c_str());
    uint32_t optionCount = options.size();
    int32_t myLength = length - sizeof(int32_t);

    uint8_t* ptr = start;
    SAVE(myLength);
    ptr = vis.save(ptr);
    SAVE(optionCount);

    for (auto& it : optionList) {
        ptr = it->save(ptr);
    }

    return ptr;
}

void OptionSet::setOption(const std::string& name, const uint8_t* buffer, int len) {
    auto it = options.find(name);
    if (it != options.end()) {
        it->second->setValue(buffer, len);
    }
}
}  // namespace NaoControl
