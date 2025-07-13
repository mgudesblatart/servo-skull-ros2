// Basic unit tests for the WebRTC VAD C++ wrapper
// If this fails, either the wrapper is broken or humanity is doomed to a future of endless static.

#include <gtest/gtest.h>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "microphone_node/webrtc_vad_wrapper.hpp"
#include <vector>

// Helper: generate a buffer of silence (all zeros)
static std::vector<int16_t> make_silence(size_t n) {
    return std::vector<int16_t>(n, 0);
}

// Helper: generate a buffer of fake speech (simple sine wave)
static std::vector<int16_t> make_speech(size_t n) {
    std::vector<int16_t> buf(n);
    for (size_t i = 0; i < n; ++i) {
        buf[i] = static_cast<int16_t>(1000 * sin(2 * M_PI * i / 32));
    }
    return buf;
}

TEST(WebrtcVad, DetectsSilence) {
    WebrtcVad vad;
    auto silence = make_silence(480); // 10ms at 48kHz
    EXPECT_FALSE(vad.is_speech(silence.data(), silence.size(), 48000));
}

TEST(WebrtcVad, DetectsSpeech) {
    WebrtcVad vad;
    auto speech = make_speech(480);
    EXPECT_TRUE(vad.is_speech(speech.data(), speech.size(), 48000));
}

TEST(WebrtcVad, HandlesBadInput) {
    WebrtcVad vad;
    // Null pointer
    EXPECT_FALSE(vad.is_speech(nullptr, 480, 48000));
    // Zero length
    auto silence = make_silence(0);
    EXPECT_FALSE(vad.is_speech(silence.data(), 0, 48000));
}
