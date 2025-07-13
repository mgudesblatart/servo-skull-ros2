#include "microphone_node/microphone_node.hpp"
#include "microphone_node/webrtc_vad_wrapper.hpp"
#include <stdexcept>
#include <iostream>

// Increase queue size to 50
static constexpr size_t MAX_QUEUE_SIZE = 50;

// Helper function to print all PortAudio devices
void print_portaudio_devices()
{
    int numDevices = Pa_GetDeviceCount();
    if (numDevices < 0)
    {
        std::cerr << "PortAudio error: " << Pa_GetErrorText(numDevices) << std::endl;
        return;
    }
    std::cout << "Available PortAudio input devices:" << std::endl;
    for (int i = 0; i < numDevices; ++i)
    {
        const PaDeviceInfo *info = Pa_GetDeviceInfo(i);
        if (info->maxInputChannels > 0)
        {
            std::cout << "  Index " << i << ": " << info->name << " (" << info->maxInputChannels << " channels)" << std::endl;
        }
    }
}

MicrophoneNode::MicrophoneNode() : Node("microphone_node"), stream_(nullptr), vad_(std::make_unique<WebrtcVad>())
{
    // Configurable parameters
    sample_rate_ = this->declare_parameter<int>("sample_rate", 48000);
    channels_ = this->declare_parameter<int>("channels", 1);
    frames_per_buffer_ = this->declare_parameter<int>("frames_per_buffer", 480); // 10ms at 48kHz
    bit_depth_ = this->declare_parameter<int>("bit_depth", 16);
    device_index_ = this->declare_parameter<int>("device_index", -1); // -1 = default

    audio_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("audio/raw", 10);

    if (!init_audio())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize audio input");
        throw std::runtime_error("Audio init failed");
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MicrophoneNode::timer_callback, this));
}

MicrophoneNode::~MicrophoneNode()
{
    close_audio();
}

bool MicrophoneNode::init_audio()
{
    PaError err = Pa_Initialize();
    if (err != paNoError)
    {
        RCLCPP_ERROR(this->get_logger(), "PortAudio initialization failed: %s", Pa_GetErrorText(err));
        return false;
    }

    print_portaudio_devices(); // Print devices at startup

    PaStreamParameters input_params;
    if (device_index_ < 0)
    {
        // Find the first input device with at least 1 input channel
        int numDevices = Pa_GetDeviceCount();
        input_params.device = paNoDevice;
        for (int i = 0; i < numDevices; ++i)
        {
            const PaDeviceInfo *info = Pa_GetDeviceInfo(i);
            if (info->maxInputChannels > 0)
            {
                input_params.device = i;
                break;
            }
        }
        if (input_params.device == paNoDevice)
        {
            RCLCPP_ERROR(this->get_logger(), "No input audio devices found. Plug in a mic and try again.");
            return false;
        }
    }
    else
    {
        input_params.device = device_index_;
    }
    if (input_params.device == paNoDevice)
    {
        RCLCPP_ERROR(this->get_logger(), "Selected device index %d is invalid.", device_index_);
        return false;
    }
    input_params.channelCount = channels_;
    input_params.sampleFormat = paInt16;
    input_params.suggestedLatency = Pa_GetDeviceInfo(input_params.device)->defaultLowInputLatency;
    input_params.hostApiSpecificStreamInfo = nullptr;

    running_ = true;
    err = Pa_OpenStream(
        &stream_,
        &input_params,
        nullptr,
        sample_rate_,
        frames_per_buffer_,
        paClipOff,
        &MicrophoneNode::pa_callback,
        this);
    if (err != paNoError)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open audio stream (device %d, sample_rate %d): %s. Try a different sample_rate or device_index.",
                     input_params.device, sample_rate_, Pa_GetErrorText(err));
        return false;
    }
    err = Pa_StartStream(stream_);
    if (err != paNoError)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start audio stream: %s", Pa_GetErrorText(err));
        return false;
    }
    audio_buffer_.resize(frames_per_buffer_ * channels_ * (bit_depth_ / 8));
    return true;
}

int MicrophoneNode::pa_callback(const void *input, void *output, unsigned long frameCount,
                                const PaStreamCallbackTimeInfo *, PaStreamCallbackFlags statusFlags, void *userData)
{
    auto *self = static_cast<MicrophoneNode *>(userData);
    if (!self->running_)
        return paContinue;
    if (!input)
    {
        RCLCPP_WARN(rclcpp::get_logger("microphone_node"), "PortAudio callback: input buffer is null (possible over/underflow)");
        return paContinue;
    }
    if (statusFlags & paInputOverflow)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("microphone_node"), "PortAudio callback: input overflow detected");
    }
    size_t bytes = frameCount * self->channels_ * (self->bit_depth_ / 8);
    std::vector<uint8_t> buf((uint8_t *)input, (uint8_t *)input + bytes);
    {
        std::lock_guard<std::mutex> lock(self->queue_mutex_);
        self->audio_queue_.push(std::move(buf));
        if (self->audio_queue_.size() > MAX_QUEUE_SIZE)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("microphone_node"), "Audio queue full, dropping oldest buffer");
            self->audio_queue_.pop();
        }
    }
    return paContinue;
}

void MicrophoneNode::close_audio()
{
    running_ = false;
    if (stream_)
    {
        Pa_StopStream(stream_);
        Pa_CloseStream(stream_);
        stream_ = nullptr;
    }
    Pa_Terminate();
}

void MicrophoneNode::timer_callback()
{
    std::vector<std::vector<uint8_t>> buffers;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        while (!audio_queue_.empty()) {
            buffers.push_back(std::move(audio_queue_.front()));
            audio_queue_.pop();
        }
    }
    if (buffers.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "No audio data in queue to publish");
        return;
    }
    for (auto& buf : buffers) {
        if (buf.size() >= sizeof(int16_t) * 10)
        {
            const int16_t *pcm = reinterpret_cast<const int16_t *>(buf.data());
            RCLCPP_DEBUG(this->get_logger(), "First 10 samples: %s %s %s %s %s %s %s %s %s %s",
                std::to_string(pcm[0]).c_str(), std::to_string(pcm[1]).c_str(), std::to_string(pcm[2]).c_str(),
                std::to_string(pcm[3]).c_str(), std::to_string(pcm[4]).c_str(), std::to_string(pcm[5]).c_str(),
                std::to_string(pcm[6]).c_str(), std::to_string(pcm[7]).c_str(), std::to_string(pcm[8]).c_str(),
                std::to_string(pcm[9]).c_str());
        }
        RCLCPP_DEBUG(this->get_logger(), "Samples: %zu, sample_rate_ %d",
                     buf.size() / sizeof(int16_t), sample_rate_);
        // VAD: Only publish if speech is detected
        if (buf.size() < sizeof(int16_t))
            continue; // not enough data
        size_t samples = buf.size() / sizeof(int16_t);
        const int16_t *pcm = reinterpret_cast<const int16_t *>(buf.data());
        bool speech = vad_->is_speech(pcm, samples, sample_rate_);
        RCLCPP_DEBUG(this->get_logger(), "VAD: Speech detected: %s", speech ? "true" : "false");
        if (!speech)
        {
            RCLCPP_DEBUG(this->get_logger(), "VAD: No speech detected, dropping buffer");
            continue;
        }
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data = std::move(buf);
        RCLCPP_DEBUG(this->get_logger(), "Publishing audio buffer of size %zu (speech detected)", msg.data.size());
        audio_pub_->publish(msg);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MicrophoneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
