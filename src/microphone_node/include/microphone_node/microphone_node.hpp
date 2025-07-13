#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <portaudio.h>
#include <vector>
#include <queue>
#include <mutex>
#include <atomic>
#include "microphone_node/webrtc_vad_wrapper.hpp"
#include <memory>

class MicrophoneNode : public rclcpp::Node {
public:
    MicrophoneNode();
    ~MicrophoneNode();

private:
    void timer_callback();
    bool init_audio();
    void close_audio();

    static int pa_callback(const void *input, void *output, unsigned long frameCount,
                           const PaStreamCallbackTimeInfo *timeInfo, PaStreamCallbackFlags statusFlags, void *userData);

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr audio_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    PaStream *stream_;
    int sample_rate_;
    int channels_;
    int frames_per_buffer_;
    int bit_depth_;
    int device_index_;
    std::vector<uint8_t> audio_buffer_;

    std::queue<std::vector<uint8_t>> audio_queue_;
    std::mutex queue_mutex_;
    std::atomic<bool> running_;
    std::unique_ptr<WebrtcVad> vad_;
};
