#include <string.h>
#include "bleutooth_stream_a2dp.h"
#include "esp_a2dp_api.h"

static struct{
    bool streaming;
    uint32_t sample_index;
    float phase_increment;
}audio_state = {0};


static const char *TAG = "A2DP_STREAM";

/*
 * @brief Start audio streaming
 * 
 * Initializes audio generation and starts the streaming process.
 * Call this after A2DP connection is established.
 */
esp_err_t start_audio_streming(void){
    if(audio_state.streaming){
        ESP_LOGW(TAG, "Audio streaming already active");
        return ESP_ERR_INVALID_STATE;
    }

    audio_state.streaming = true;

    ESP_LOGI(TAG, "Audio streaming started - ready to transmit data");
    return ESP_OK;
}

/**
 * @brief Stop audio streaming
 */
void stop_audio_streaming(void)
{
    audio_state.streaming = false;
    audio_state.sample_index = 0;  // Reset for next session
    ESP_LOGI(TAG, "Audio streaming stopped");
}

/**
 * @brief Check if audio streaming is active
 * 
 * @return true if streaming is active, false otherwise
 */
bool is_audio_streaming_active(void)
{
    return audio_state.streaming;
}

/**
 * @brief Get current sample index for phase continuity
 * 
 * Useful for audio generators that need to maintain phase
 * continuity across multiple audio chunks.
 * 
 * @return Current sample index
 */
uint32_t get_current_sample_index(void)
{
    return audio_state.sample_index;
}


/**
 * @brief Calculate optimal buffer size for audio streaming
 * 
 * Helper function to calculate buffer size based on audio parameters.
 * 
 * @param samples_per_chunk Number of stereo samples per chunk
 * @param channels Number of audio channels (1=mono, 2=stereo)  
 * @param bits_per_sample Bit depth (typically 16)
 * @return Size in bytes for the audio buffer
 */
size_t calculate_audio_buffer_size(size_t samples_per_chunk, uint8_t channels, uint8_t bits_per_sample)
{
    return samples_per_chunk * channels * (bits_per_sample / 8);
}

/**
 * @brief A2DP data request callback helper with configurable parameters
 * 
 * Flexible function for handling A2DP data requests. Takes audio format
 * parameters.
 * 
 * @param samples_per_chunk Number of samples to generate per channel
 * @param channels Number of audio channels (1=mono, 2=stereo, etc.)
 * @param bits_per_sample Bit depth (8, 16, 24, 32)
 * @param audio_generator Function pointer to generate audio data
 * @param generator_context Context data for the audio generator
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t handle_a2dp_data_request(size_t samples_per_chunk,
                                   uint8_t channels,
                                   uint8_t bits_per_sample,
                                   void(*audio_generator)(void *buffer, size_t samples, uint8_t channelss, uint32_t sample_offset, void *context),
                                   void *generator_context){
    if(!audio_state.streaming){
        return ESP_ERR_INVALID_STATE;
    }

    // Calculate buffer size based on actual parameters
    size_t buffer_size = calculate_audio_buffer_size(samples_per_chunk, channels, bits_per_sample);

    // Allocate buffer for this chunk
    void *audio_buffer = malloc(buffer_size);
    if(!audio_buffer){
        ESP_LOGE(TAG, "Failed to allocate %zu byte audio buffer", buffer_size);
        return ESP_ERR_NO_MEM;
    }

    // Generate audio using provided generator function
    if(audio_generator){
        audio_generator(audio_buffer, samples_per_chunk, channels, audio_state.sample_index, generator_context);
    }else{
        // Fill with silence
        memset(audio_buffer, 0, buffer_size);
    }

    esp_err_t ret = send_audio_chunk((uint8_t*) audio_buffer, buffer_size);

    // Update sample index
    if(ret == ESP_OK){
        audio_state.sample_index += samples_per_chunk;
    }

    free(audio_buffer);
    return ESP_OK;
}