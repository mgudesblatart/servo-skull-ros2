/*
 * Minimal C++ wrapper for WebRTC VAD (just make it work edition)
 * Place this in your microphone_node/include/microphone_node/webrtc_vad_wrapper.hpp
 */
#pragma once
extern "C" {
#include "webrtc_vad/include/webrtc_vad.h"
}
#include <vector>
#include <cstdint>

class WebrtcVad {
public:
    WebrtcVad() {
        handle_ = WebRtcVad_Create();
        WebRtcVad_Init(handle_);
        // 3 = most aggressive (less false positives, more false negatives)
        WebRtcVad_set_mode(handle_, 2);
    }
    ~WebrtcVad() {
        WebRtcVad_Free(handle_);
    }
    // Returns true if speech is detected
    bool is_speech(const int16_t* data, size_t samples, int sample_rate) {
        if (!data || samples == 0) return false; // treat empty/null as not speech
        // WebRTC VAD only supports 10, 20, or 30ms frames
        int valid = (samples == sample_rate/100 || samples == sample_rate/50 || samples == sample_rate/33);
        if (!valid) return true; // fallback: always publish if frame size is weird
        int r = WebRtcVad_Process(handle_, sample_rate, data, samples);
        return r == 1;
    }
private:
    VadInst* handle_;
};
