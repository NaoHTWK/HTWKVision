#ifndef EXPONENTIAL_MOVING_AVERAGE_H
#define EXPONENTIAL_MOVING_AVERAGE_H

#include <functional>

// Exponential moving average.
// The += operator does:
// val = val * factor + new_val * (1.f - factor)

// Usage examples:
// EMA something(0.f, 0.99f);
// something.setFactor(0.98f);
// something+= 0.5f;

// EMA is implicitly convertible to float:
// float f = something;
// if (something > 0.1f) { ... }

// If you need an std::function that updates the factor (e.g. when using the Options framework) try this:
// EMA::update(&something)

class EMA {
public:
    EMA(float val, float factor) : val(val), factor(factor) {}

    void setFactor(float new_factor) {
        factor = new_factor;
    }

    EMA& operator+=(float new_val) {
        val = val * factor + new_val * (1.f - factor);  // NOLINT
        return *this;
    }

    operator float() const { return val; }

    static std::function<void(float)> update(EMA* ema) {
        return std::bind(&EMA::setFactor, ema, std::placeholders::_1);
    }

private:
    float val;
    float factor;
};

#endif // EXPONENTIAL_MOVING_AVERAGE_H
