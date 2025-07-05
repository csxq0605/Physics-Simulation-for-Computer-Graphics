#pragma once

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>

template<typename Callable>
void parallel_for(int n, Callable function, int num_threads = 0) {
    if (n <= 0) return;

    // const int hardware_threads = std::thread::hardware_concurrency();
    // const int actual_threads = (num_threads > 0) ? 
    //     std::min(num_threads, n) : 
    //     std::min(hardware_threads != 0 ? hardware_threads : 2, n);
    const int actual_threads = 8;

    if (actual_threads == 1) {
        for (int i = 0; i < n; ++i) {
            function(i);
        }
        return;
    }

    const int base_chunk_size = n / actual_threads;
    const int remainder = n % actual_threads;

    std::vector<std::thread> threads;
    threads.reserve(actual_threads);

    for (int thread_id = 0; thread_id < actual_threads; ++thread_id) {
        const int start = thread_id * base_chunk_size + std::min(thread_id, remainder);
        const int end = start + base_chunk_size + (thread_id < remainder ? 1 : 0);

        threads.emplace_back([=, &function] {
            for (int i = start; i < end; ++i) {
                function(i);
            }
        });
    }

    for (auto &thread : threads) {
        thread.join();
    }
}
