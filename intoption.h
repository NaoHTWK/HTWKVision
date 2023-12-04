#ifndef INTOPTION_H_
#define INTOPTION_H_

#include <option.h>

#include <climits>
#include <cstdint>
#include <limits>

using std::numeric_limits;

namespace NaoControl {

template <typename T>
class IntOption : public Option {
private:
    T* value;
    std::function<T(void)> getCallback;
    std::function<void(T)> setCallback;

public:
    /** This class is not intended to by copied */
    IntOption(IntOption& s) = delete;
    IntOption(IntOption&& s) = delete;
    IntOption& operator=(const IntOption&) = delete;
    IntOption& operator=(IntOption&&) = delete;
    ~IntOption() override = default;

    IntOption(const char* cname, T min, T max, T step, std::function<T(void)> getCallback,
              std::function<void(T)> setCallback);
    IntOption(const char* cname, T* val, T min = numeric_limits<T>::min(), T max = numeric_limits<T>::max(), T step = 1,
              std::function<void(T)> callb = nullptr);

    uint8_t* save(uint8_t* start) override;
    void setValue(const uint8_t* buffer, int len) override;
    OptionType getOptionType() override {
        return O_INT;
    }
    void* getValuePtr() override {
        return value;
    }
};
}  // namespace NaoControl

#endif /* INTOPTION_H_ */
