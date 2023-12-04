#pragma once

#include <cstddef>
#include <vector>

#include <stl_ext.h>

template <typename T>
class Raster {
public:
    size_t width;
    size_t height;

    Raster(size_t width, size_t height) : width(width), height(height), data_(width * height) {}
    Raster(size_t width, size_t height, const T& default_val)
        : width(width), height(height), data_(width * height, default_val) {}

    T* data() {
        return data_.data();
    }
    const T* const data() const {
        return data_.data();
    }
    T& operator()(int x, int y) {
        return data_[x + y * width];
    }
    const T& operator()(int x, int y) const {
        return data_[x + y * width];
    }
    T& clamped(int x, int y) {
        return data_[clamp(x, 0, width - 1) + clamp(y, 0, height - 1) * width];
    }
    const T& clamped(int x, int y) const {
        return data_[clamp(x, 0, width - 1) + clamp(y, 0, height - 1) * width];
    }
    bool inside(int x, int y) const {
        return within(x, 0, width - 1) && within(y, 0, height - 1);
    }
    auto begin() {
        return data_.begin();
    }
    auto begin() const {
        return data_.begin();
    }
    auto cbegin() const {
        return data_.cbegin();
    }
    auto end() {
        return data_.end();
    }
    auto end() const {
        return data_.end();
    }
    auto cend() const {
        return data_.cend();
    }

private:
    std::vector<T> data_;
};
