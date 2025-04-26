#ifndef WEBRTC_APM_H
#define WEBRTC_APM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to WebRTC AudioProcessing module
typedef void* apm_handle_t;

/**
 * @brief Create AudioProcessing instance
 * @param sample_rate Sampling rate in Hz (e.g., 16000)
 * @param num_capture_channels Number of capture (mic) channels
 * @param num_reverse_channels Number of reverse (ref) channels
 * @return apm_handle_t or NULL on failure
 */
apm_handle_t apm_create(int sample_rate, int num_capture_channels, int num_reverse_channels);

/**
 * @brief Destroy AudioProcessing instance
 */
void apm_destroy(apm_handle_t handle);

/**
 * @brief Process reverse (reference) stream for echo canceller
 * @param handle APM handle
 * @param data Pointer to interleaved mono reverse samples
 * @param samples_per_channel Number of samples in data
 */
void apm_process_reverse(apm_handle_t handle, int16_t* data, size_t samples_per_channel);

/**
 * @brief Process captured (microphone) stream
 * @param handle APM handle
 * @param data Pointer to interleaved capture samples
 * @param samples_per_channel Number of samples in data
 */
void apm_process_stream(apm_handle_t handle, int16_t* data, size_t samples_per_channel);

#ifdef __cplusplus
}
#endif

#endif // WEBRTC_APM_H
