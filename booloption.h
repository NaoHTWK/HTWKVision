#ifndef BOOLOPTION_H_
#define BOOLOPTION_H_

#include "option.h"

namespace NaoControl {

class BoolOption : public NaoControl::Option {
private:
    bool* value;
    std::function<void(bool)> callback;

public:
    /** This class is not intended to by copied */
    BoolOption(BoolOption& s) = delete;
    BoolOption(BoolOption&& s) = delete;
    BoolOption& operator=(const BoolOption&) = delete;
    BoolOption& operator=(BoolOption&&) = delete;
    ~BoolOption() override = default;

    BoolOption(const char* cname, bool* val, std::function<void(bool)> callb = nullptr);
    uint8_t* save(uint8_t* start) override;
    void setValue(const uint8_t* buffer, int len) override;
    OptionType getOptionType() override {
        return O_BOOL;
    }
    void* getValuePtr() override {
        return value;
    }
};

} /* namespace NaoControl */
#endif /* BOOLOPTION_H_ */
