#ifndef OPTION_H_
#define OPTION_H_

#include <climits>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>

namespace NaoControl {

class Option {
protected:
    const std::string name;

    uint8_t* buffer;
    uint32_t length;

    Option(const char* n);

public:
    /** This class is not intended to by copied */
    Option(Option& s) = delete;
    Option(Option&& s) = delete;
    Option& operator=(const Option&) = delete;
    Option& operator=(Option&&) = delete;

    enum OptionType { O_INT = 0, O_FLOAT, O_BOOL, O_ENUM };

    virtual ~Option() {
        free(buffer);
    }
    virtual uint8_t* save(uint8_t* start);
    virtual size_t size() const {
        return length;
    }
    virtual void setValue(const uint8_t* buffer, int len) = 0;

    virtual const std::string& getName() {
        return name;
    }
    virtual OptionType getOptionType() = 0;
    virtual void* getValuePtr() = 0;
};

using OptionPtr = std::shared_ptr<Option>;

} /* namespace NaoControl */
#endif /* OPTION_H_ */
