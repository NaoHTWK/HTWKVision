#pragma once

#include <stl_ext.h>
#include <threadsafe_deque.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <iostream>
#include <list>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

class ExecutionState {
public:
    ExecutionState(std::condition_variable* cv, std::mutex* mtx) : cv(cv), mtx(mtx) {}

    bool isFinished() const {
        return finished;
    }
    void setFinshed() {
        std::lock_guard<std::mutex> lck(*mtx);
        finished = true;
        cv->notify_all();
    }

private:
    std::atomic<bool> finished{false};
    std::condition_variable* cv;
    std::mutex* mtx;
};

class ThreadPool {
public:
    // If num_threads isn't set (or set to 0), number of threads is determined by hardware concurrency.
    ThreadPool(const std::string& name, bool low_prio = false, uint32_t num_threads = 0) {
        if (num_threads == 0) {
            num_threads = std::thread::hardware_concurrency();
            if (num_threads == 0) {
                std::cerr << "Your compiler doesn't support std::thread::hardware_concurrency properly, get a new one."
                          << std::endl;
                exit(-4);
            }
        }
        for (uint32_t i = 0; i < num_threads; i++) {
            launch_named_thread(name + "_" + std::to_string(i), low_prio, [this]() {
                while (true)
                    tasks.pop_front()();
            }).detach();
        }
    }

    template <typename T>
    std::shared_ptr<std::packaged_task<T()>> run(std::function<T()> f) {
        std::shared_ptr<std::packaged_task<T()>> task(new std::packaged_task<T()>(f));
        tasks.push_back(std::packaged_task<void()>([task]() { (*task)(); }));
        return task;
    }

    template <typename T>
    std::shared_ptr<std::packaged_task<T()>> run(std::function<T()> f, ExecutionState* state) {
        std::shared_ptr<std::packaged_task<T()>> task(new std::packaged_task<T()>(f));
        tasks.push_back(std::packaged_task<void()>([task, state]() {
            (*task)();
            state->setFinshed();
        }));
        return task;
    }

private:
    ThreadSafeDeque<std::packaged_task<void()>> tasks;
};

class TaskScheduler {
public:
    TaskScheduler(ThreadPool* pool) : pool(pool) {}

    ExecutionState* addTask(std::function<void()> task, std::vector<ExecutionState*> dependencies) {
        std::lock_guard<std::mutex> lck(mtx);
        states.emplace_back(&cv, &mtx);
        tasks_to_schedule.emplace_back(task, &states.back(), dependencies);
        return &states.back();
    }
    void run() {
        while (true) {
            std::unique_lock<std::mutex> lck(mtx);
            bool ran_task = false;
            for (auto it = tasks_to_schedule.begin(); it != tasks_to_schedule.end();) {
                if (depsFinished(std::get<2>(*it))) {
                    pool->run(std::get<0>(*it), std::get<1>(*it));
                    ran_task = true;
                    it = tasks_to_schedule.erase(it);
                } else {
                    it++;
                }
            }
            if (ran_task)
                continue;
            if (allTasksFinished())
                break;
            cv.wait(lck);
        }
    }

private:
    static bool depsFinished(const std::vector<ExecutionState*>& deps) {
        for (const auto& dep : deps)
            if (!dep->isFinished())
                return false;
        return true;
    }
    bool allTasksFinished() const {
        for (const auto& state : states)
            if (!state.isFinished())
                return false;
        return true;
    }

    ThreadPool* pool;
    std::condition_variable cv;
    std::mutex mtx;
    std::list<ExecutionState> states;
    std::list<std::tuple<std::function<void()>, ExecutionState*, std::vector<ExecutionState*>>> tasks_to_schedule;
};

// AsyncCallback is meant for functions that should be repeatedly executed asynchronously without generating a new
// thread for each execution.
// Guarantees:
//  - func will never run in parallel to itself
//  - invocations of func will happen in the same order as calls to operator()
//  - if more than queue_length executions are waiting, the oldest executions will be dropped
template <typename... Args>
class AsyncCallback {
public:
    AsyncCallback(const std::string& thread_name, std::function<void(Args...)> func, size_t queue_length = 1)
        : tasks(std::make_unique<ThreadSafeDeque<std::function<void(std::function<void(Args...)>)>>>(queue_length)),
          running(std::make_unique<std::atomic_bool>(true)),
          thread(launch_named_thread(thread_name, false, 
                                     [running = running.get(), tasks = tasks.get(), func = std::move(func)]() {
                                         while (*running)
                                             tasks->pop_front()(func);
                                     })),
          thread_name(thread_name) {}

    ~AsyncCallback() {
        // Don't delete anything if it's been moved.
        if (running == nullptr || tasks == nullptr) {
            return;
        }
        *running = false;
        // Add another element that doesn't actually execute anything to the queue in case the thread is waiting.
        tasks->push_back([](std::function<void(Args...)>) {});
        thread.join();
    }

    AsyncCallback(AsyncCallback&) = delete;
    AsyncCallback& operator=(AsyncCallback&) = delete;

    AsyncCallback(AsyncCallback&&) noexcept = default;
    AsyncCallback& operator=(AsyncCallback&&) noexcept = default;

    void operator()(Args... args) {
        tasks->push_back([args...](std::function<void(Args...)> f) { f(args...); });
    }

    std::string name() const { return thread_name; }

private:
    std::unique_ptr<ThreadSafeDeque<std::function<void(std::function<void(Args...)>)>>> tasks;
    std::unique_ptr<std::atomic_bool> running;
    std::thread thread;
    std::string thread_name;
};
