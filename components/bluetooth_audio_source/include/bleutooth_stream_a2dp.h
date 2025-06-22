#include <math.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_err.h"


esp_err_t start_aduio_streaming(void);
void stop_audio_streaming_active(void);
esp_err_t send_audio_chunk(const uint8_t *audio_data, size_t data_size);

bool is_audio_streaming_active(void);
uint32_t get_current_sample_index(void);

size_t calculate_audio_buffer_size(size_t samples_per_chunk, uint8_t channels, uint8_t bits_per_sample);

esp_err_t handle_a2dp_data_request(size_t samples_per_chunk,
                                   uint8_t channels,
                                   uint8_t bits_per_sample,
                                   void(*audio_generator)(void *buffer, size_t samples, uint8_t channelss, uint32_t sample_offset, void *context),
                                   void *generator_context);