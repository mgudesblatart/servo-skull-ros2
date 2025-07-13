// Unit tests for the thread-safe audio queue used in microphone_node
// If this queue breaks, so does the illusion of real-time audio. No pressure.

#include <gtest/gtest.h>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>
// #include "microphone_node/microphone_node.hpp" // Assuming queue is defined here or in a sub-header

// Dummy audio buffer type for testing
using AudioBuffer = std::vector<int16_t>;

// Minimal mock of the queue (replace with actual queue class if available)
class AudioQueue {
public:
    AudioQueue(size_t max_size) : max_size_(max_size) {}
    bool push(const AudioBuffer& buf) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= max_size_) return false;
        queue_.push_back(buf);
        return true;
    }
    bool pop(AudioBuffer& out) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) return false;
        out = queue_.front();
        queue_.erase(queue_.begin());
        return true;
    }
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
private:
    size_t max_size_;
    mutable std::mutex mutex_;
    std::vector<AudioBuffer> queue_;
};

TEST(AudioQueue, PushPopBasic) {
    AudioQueue q(3);
    AudioBuffer buf1(10, 1), buf2(10, 2);
    EXPECT_TRUE(q.push(buf1));
    EXPECT_TRUE(q.push(buf2));
    AudioBuffer out;
    EXPECT_TRUE(q.pop(out));
    EXPECT_EQ(out, buf1);
    EXPECT_TRUE(q.pop(out));
    EXPECT_EQ(out, buf2);
    EXPECT_FALSE(q.pop(out)); // empty
}

TEST(AudioQueue, Overflow) {
    AudioQueue q(2);
    AudioBuffer buf(10, 1);
    EXPECT_TRUE(q.push(buf));
    EXPECT_TRUE(q.push(buf));
    EXPECT_FALSE(q.push(buf)); // should overflow
}

TEST(AudioQueue, ThreadSafety) {
    AudioQueue q(100);
    std::atomic<bool> done{false};
    std::thread producer([&]() {
        for (int i = 0; i < 1000; ++i) {
            AudioBuffer buf(10, i);
            while (!q.push(buf)) std::this_thread::yield();
        }
        done = true;
    });
    std::thread consumer([&]() {
        AudioBuffer out;
        int count = 0;
        while (!done || q.size() > 0) {
            if (q.pop(out)) ++count;
            else std::this_thread::yield();
        }
        EXPECT_GE(count, 1000);
    });
    producer.join();
    consumer.join();
}
