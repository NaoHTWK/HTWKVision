#ifndef FLOATOPTION_H_
#define FLOATOPTION_H_

#include <option.h>

#include <cfloat>
#include <climits>
#include <cstdint>

namespace NaoControl {

class FloatOption : public Option {
private:
    float* value;
    std::function<void(float)> callback;

public:
    /** This class is not intended to by copied */
    FloatOption(FloatOption& s) = delete;
    FloatOption(FloatOption&& s) = delete;
    FloatOption& operator=(const FloatOption&) = delete;
    FloatOption& operator=(FloatOption&&) = delete;
    ~FloatOption() override = default;

    FloatOption(const char* cname, float* val, float min = FLT_MIN, float max = FLT_MAX, float step = 0.1f,
                std::function<void(float)> callb = nullptr);
    uint8_t* save(uint8_t* start) override;
    void setValue(const uint8_t* buffer, int len) override;
    OptionType getOptionType() override {
        return O_FLOAT;
    }
    void* getValuePtr() override {
        return value;
    }
};

}  // namespace NaoControl

#endif /* FLOATOPTION_H_ */
