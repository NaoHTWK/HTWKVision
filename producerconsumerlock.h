#ifndef PRODUCERCONSUMERLOCK_H
#define PRODUCERCONSUMERLOCK_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <list>

#include <pthread.h>

namespace FlightRecorder {
namespace Thread {

class ProducerConsumerLock {
private:
    pthread_cond_t producerCondition;
    pthread_cond_t consumerCondition;
    pthread_mutex_t mutex;

    std::function<bool()> isEmpty;

    std::atomic<uint64_t> queueHead{0};
    std::atomic<uint64_t> queueTail{0};

public:
    explicit ProducerConsumerLock();
    ~ProducerConsumerLock();

    void init(std::function<bool()> _isEmpty);

    void producerLock();
    void producerUnlock();
    void consumerLock();
    void consumerUnlock();
};

}  // namespace Thread
}  // namespace FlightRecorder

#endif  // PRODUCERCONSUMERLOCK_H
