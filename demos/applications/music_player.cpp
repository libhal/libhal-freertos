// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_SIMD

#include <FreeRTOS.h>
#include <minimp3.h>
#include <queue.h>
#include <task.h>

#include <libhal-freertos/task.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/static_callable.hpp>
#include <libhal-util/steady_clock.hpp>

#include "../resource_list.hpp"
#include "infraction_bam_bam_royalty_free_16kHz_mono.mp3.h"

using sample_t = hal::stream_dac_u8::samples;

resource_list hardware_map;
QueueHandle_t sample_queue = NULL;
unsigned int audio_streamer_percent = 0;
unsigned int audio_decoder_percent = 0;

void audio_streamer(void*) noexcept
{
  while (true) {
    audio_streamer_percent =
      ulTaskGetRunTimePercent(xTaskGetCurrentTaskHandle());
    sample_t current_samples{};
    xQueueReceive(sample_queue, &current_samples, portMAX_DELAY);
    (*hardware_map.dac)->write(current_samples);
  }
}

std::array<std::int16_t, MINIMP3_MAX_SAMPLES_PER_FRAME> pcm16;
mp3dec_t mp3d;

std::span<std::uint8_t> pcm16_to_pcm8(std::span<std::int16_t> p_input,
                                      std::span<std::uint8_t> p_output)
{
  auto const min_transfer = std::min(p_input.size(), p_output.size());

  for (size_t j = 0; j < min_transfer; j++) {
    constexpr std::uint16_t positive_offset = (UINT16_MAX / 2) + 1;
    auto const unsigned_value = static_cast<std::uint16_t>(p_input[j]);
    std::int16_t const unsigned_pcm16_value = unsigned_value + positive_offset;
    p_output[j] = static_cast<std::uint8_t>(unsigned_pcm16_value >> 8);
  }

  return p_output.subspan(min_transfer);
}

void audio_decoder(void*) noexcept
{
  mp3dec_init(&mp3d);
  std::span<hal::byte const> mp3_data(
    infraction_bam_bam_royalty_free_16kHz_mono_mp3);
  size_t buffer_index = 0;
  std::array pcm8_array{
    hardware_map.pcm8_buffer1,
    hardware_map.pcm8_buffer2,
  };
  std::span<hal::byte> pcm8_span(pcm8_array[buffer_index]);
  std::ranges::fill(pcm8_array[0], 0);
  std::ranges::fill(pcm8_array[1], 0);

  while (true) {
    audio_decoder_percent =
      ulTaskGetRunTimePercent(xTaskGetCurrentTaskHandle());
    mp3dec_frame_info_t info{};

    std::size_t sample_count = mp3dec_decode_frame(
      &mp3d, mp3_data.data(), mp3_data.size(), pcm16.data(), &info);

    mp3_data = mp3_data.subspan(info.frame_bytes);

    if (mp3_data.empty()) {
      mp3_data = infraction_bam_bam_royalty_free_16kHz_mono_mp3;
    }

    size_t span_size = pcm8_span.size();
    pcm8_span = pcm16_to_pcm8({ pcm16.data(), sample_count }, pcm8_span);

    if (pcm8_span.empty()) {
      sample_t finished_samples = {
        .sample_rate = static_cast<float>(info.hz) * 0.85f,
        .data = { pcm8_array[buffer_index] },
      };

      buffer_index = (buffer_index + 1) % pcm8_array.size();
      pcm8_span = pcm8_array[buffer_index];

      // If we have remaining data, we need to move it into the other buffer
      if (span_size < sample_count) {
        pcm8_span =
          pcm16_to_pcm8({ pcm16.data(), sample_count - span_size }, pcm8_span);
      }

      (*hardware_map.led)->level(!(*hardware_map.led)->level());
      xQueueSend(sample_queue, &finished_samples, portMAX_DELAY);
    }
  }
}

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  hardware_map = p_map;

  static StaticQueue_t static_queue;
  constexpr size_t queue_size = 1;
  static std::array<hal::byte, sizeof(sample_t) * queue_size>
    queue_storage_area;

  sample_queue = xQueueCreateStatic(
    queue_size, sizeof(sample_t), queue_storage_area.data(), &static_queue);

  static hal::freertos::static_task audio_streamer_task(
    "stream", audio_streamer, hal::buffer<128 * sizeof(StackType_t)>);

  static hal::freertos::static_task audio_decoder_task(
    "decoder", audio_decoder, hal::buffer<6000 * sizeof(StackType_t)>);

  vTaskStartScheduler();

  // The bulk of the stack space used by the decoder is due to the
  // mp3dec_scratch_t being placed on the stack when you perform the mp3 frame
  // decode. This is just here for LSP inspection.
  [[maybe_unused]] constexpr size_t mp3dec_scratch_t_size =
    sizeof(mp3dec_scratch_t);
}
