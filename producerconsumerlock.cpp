#include "producerconsumerlock.h"

#include <cstdio>
#include <cstdlib>
#include <utility>

namespace FlightRecorder {
namespace Thread {

ProducerConsumerLock::ProducerConsumerLock() : queueHead(0), queueTail(0)
{
//    printf("ProducerConsumerLock::constructor was called: %p\n", this);

    if(pthread_cond_init(&producerCondition, nullptr) != 0)
    {
        perror("ProducerConsumerLock: Can't init producerCondition.");
        exit(1);
    }

    if(pthread_cond_init(&consumerCondition, nullptr) != 0)
    {
        perror("ProducerConsumerLock: Can't init consumerCondition.");
        exit(1);
    }

    if(pthread_mutex_init(&mutex, nullptr) != 0)
    {
        perror("ProducerConsumerLock: Can't init mutex.");
        exit(1);
    }
}

ProducerConsumerLock::~ProducerConsumerLock()
{
    pthread_cond_destroy(&producerCondition);
    pthread_cond_destroy(&consumerCondition);
    pthread_mutex_destroy(&mutex);
}

void ProducerConsumerLock::init(std::function<bool ()> _isEmpty)
{
//    printf("ProducerConsumerLock::init was called: %p\n", this);

    isEmpty = std::move(_isEmpty);
}


void ProducerConsumerLock::producerLock()
{
    if(pthread_mutex_lock(&mutex) != 0)
    {
        perror("ProducerConsumerLock::producerLock: Error aquiring mutex.");
        exit(1);
    }

    uint64_t myQueueNumber = queueTail.fetch_add(1);
    while(myQueueNumber != queueHead)
    {
//        printf("waiting: %lu %lu\n", myQueueNumber, queueHead);
        if(pthread_cond_wait(&producerCondition, &mutex))
        {
            perror("ProducerConsumerLock::producerLock: Error waiting for condition.");
            exit(1);
        }
    }

//    printf("waiting: done\n");
}

void ProducerConsumerLock::producerUnlock()
{
//    printf("waiting: incrementing %llu\n",queueHead+1);

    queueHead++;

    if(pthread_cond_broadcast(&producerCondition) != 0)
    {
        perror("ProducerConsumerLock::producerUnlock: Error broadcasting signal.");
        exit(1);
    }

    if(pthread_cond_signal(&consumerCondition) != 0)
    {
        perror("ProducerConsumerLock::producerUnlock: Error setting consumer signal.");
        exit(1);
    }

    if(pthread_mutex_unlock(&mutex) != 0)
    {
        perror("ProducerConsumerLock::producerLock: Error releasing mutex.");
        exit(1);
    }
}

void ProducerConsumerLock::consumerLock()
{
    if(pthread_mutex_lock(&mutex) != 0)
    {
        perror("ProducerConsumerLock::consumerLock: Error aquiring mutex.");
        exit(1);
    }

    while(isEmpty()) {
        if(pthread_cond_wait(&consumerCondition, &mutex))
        {
            perror("ProducerConsumerLock::consumerLock: Error waiting for condition.");
            exit(1);
        }
    }
}

void ProducerConsumerLock::consumerUnlock()
{
    if(pthread_mutex_unlock(&mutex) != 0)
    {
        perror("ProducerConsumerLock::consumerLock: Error releasing mutex.");
        exit(1);
    }
}

}}
