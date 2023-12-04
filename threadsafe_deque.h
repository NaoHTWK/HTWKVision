#pragma once

#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <mutex>
#include <utility>

// Because of the underlying std::deque, if T is not copy-constructible (e.g. because it's a move-only type) it needs to
// have a no-argument constructor.
template <typename T>
class ThreadSafeDeque {
public:
    ThreadSafeDeque(size_t max_size = 0) : max_size(max_size) {}

    void push_back(T& e) {
        std::unique_lock<std::mutex> lock(mutex);
        storage.push_back(e);

        if (max_size > 0) {
            while (storage.size() > max_size)
                storage.pop_front();
        }

        cv.notify_one();
        lock.unlock();
    }

    void push_back(T&& e) {
        std::unique_lock<std::mutex> lock(mutex);
        storage.push_back(std::forward<T>(e));

        if (max_size > 0) {
            while (storage.size() > max_size)
                storage.pop_front();
        }

        cv.notify_one();
        lock.unlock();
    }

    T pop_front() {
        std::unique_lock<std::mutex> lock(mutex);

        if (storage.empty())
            cv.wait(lock, [this]() { return !storage.empty(); });

        if constexpr (std::is_copy_constructible<T>::value) {
            T elem = storage.front();
            storage.pop_front();
            return elem;
        } else {
            // Support move-only types as long as they have a no-argument constructor.
            T elem;
            std::swap(*storage.begin(), elem);
            storage.pop_front();
            return elem;
        }
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex);
        return storage.empty();
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex);
        storage.clear();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex);
        return storage.size();
    }

private:
    std::deque<T> storage;
    mutable std::mutex mutex;
    std::condition_variable cv;
    size_t max_size;
};
