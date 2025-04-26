#include "webrtc_apm.h"
#include "api/audio/audio_processing.h"
#include "api/audio/builtin_audio_processing_builder.h"
#include "api/environment/environment.h"
#include "api/environment/environment_factory.h"
#include <vector>
#include <cstdint>

using webrtc::AudioProcessing;
using webrtc::BuiltinAudioProcessingBuilder;
using webrtc::CreateEnvironment;
using webrtc::scoped_refptr;
using webrtc::StreamConfig;

typedef struct APMContext {
    scoped_refptr<AudioProcessing> apm;
    int sample_rate;
    int capture_channels;
    int reverse_channels;
} APMContext;

extern "C" apm_handle_t apm_create(int sample_rate, int num_capture_channels, int num_reverse_channels) {
    APMContext* ctx = new APMContext();
    ctx->sample_rate = sample_rate;
    ctx->capture_channels = num_capture_channels;
    ctx->reverse_channels = num_reverse_channels;
    AudioProcessing::Config config;
    config.pipeline.multi_channel_capture = num_capture_channels > 1;
    config.pipeline.multi_channel_render = num_reverse_channels > 1;
    config.echo_canceller.enabled = true;
    config.noise_suppression.enabled = true;
    config.gain_controller2.enabled = true;
    config.high_pass_filter.enabled = true;
    BuiltinAudioProcessingBuilder builder(config);
    ctx->apm = builder.Build(CreateEnvironment());
    if (!ctx->apm) {
        delete ctx;
        return nullptr;
    }
    ctx->apm->Initialize();
    return reinterpret_cast<apm_handle_t>(ctx);
}

extern "C" void apm_destroy(apm_handle_t handle) {
    if (!handle) return;
    delete reinterpret_cast<APMContext*>(handle);
}

extern "C" void apm_process_reverse(apm_handle_t handle, int16_t* data, size_t samples_per_channel) {
    if (!handle) return;
    APMContext* ctx = reinterpret_cast<APMContext*>(handle);
    int channels = ctx->reverse_channels;
    StreamConfig cfg(ctx->sample_rate, channels);
    ctx->apm->ProcessReverseStream(data, cfg, cfg, nullptr);
}

extern "C" void apm_process_stream(apm_handle_t handle, int16_t* data, size_t samples_per_channel) {
    if (!handle) return;
    APMContext* ctx = reinterpret_cast<APMContext*>(handle);
    int channels = ctx->capture_channels;
    StreamConfig cfg(ctx->sample_rate, channels);
    ctx->apm->ProcessStream(data, cfg, cfg, data);
}
