#include "microphone_node/microphone_node.hpp"
#include <stdexcept>
#include <iostream>
#include <alsa/asoundlib.h>

// Increase queue size to 50
static constexpr size_t MAX_QUEUE_SIZE = 50;

// Suppress verbose ALSA probe noise from PortAudio initialization.
// Real stream open/start failures are still surfaced via explicit RCLCPP_ERROR logs.
static void alsa_error_silencer(
    const char * /*file*/,
    int /*line*/,
    const char * /*function*/,
    int /*err*/,
    const char * /*fmt*/,
    ...)
{
}

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

MicrophoneNode::MicrophoneNode() : Node("microphone_node"), stream_(nullptr)
{
    // Configurable parameters
    sample_rate_ = this->declare_parameter<int>("sample_rate", 48000);
    channels_ = this->declare_parameter<int>("channels", 1);
    frames_per_buffer_ = this->declare_parameter<int>("frames_per_buffer", 480 * 20);
    bit_depth_ = this->declare_parameter<int>("bit_depth", 16);
    device_index_ = this->declare_parameter<int>("device_index", -1); // -1 = default

    audio_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("audio/raw", 10);
    auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
    ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("/microphone_node/ready", ready_qos);

    if (!init_audio())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize audio input");
        throw std::runtime_error("Audio init failed");
    }
}

MicrophoneNode::~MicrophoneNode()
{
    close_audio();
}

bool MicrophoneNode::init_audio()
{
    snd_lib_error_set_handler(alsa_error_silencer);

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

    const PaDeviceInfo *selected_device_info = Pa_GetDeviceInfo(input_params.device);
    if (selected_device_info == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to query selected input device info for index %d.", input_params.device);
        return false;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Using input device %d: %s (%d input channels)",
        input_params.device,
        selected_device_info->name,
        selected_device_info->maxInputChannels);

    input_params.channelCount = channels_;
    input_params.sampleFormat = paInt16;
    input_params.suggestedLatency = selected_device_info->defaultLowInputLatency;
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

    RCLCPP_INFO(
        this->get_logger(),
        "Audio input stream started at %d Hz, %d channel(s), %d frames/buffer",
        sample_rate_,
        channels_,
        frames_per_buffer_);

    auto ready_msg = std_msgs::msg::Bool();
    ready_msg.data = true;
    ready_pub_->publish(ready_msg);
    RCLCPP_INFO(this->get_logger(), "Published readiness: /microphone_node/ready = true");

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
    // Directly publish audio buffer, no queue
    auto msg = std_msgs::msg::UInt8MultiArray();
    msg.data = std::move(buf);
    RCLCPP_DEBUG(self->get_logger(), "Publishing audio buffer of size %zu", msg.data.size());
    self->audio_pub_->publish(msg);
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
    snd_lib_error_set_handler(nullptr);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MicrophoneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
