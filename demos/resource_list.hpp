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

#pragma once

#include <optional>

#include <libhal/functional.hpp>
#include <libhal/io_waiter.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/stream_dac.hpp>

#include <span>

struct resource_list
{
  std::optional<hal::output_pin*> led;
  std::optional<hal::steady_clock*> clock;
  std::optional<hal::stream_dac_u8*> dac;
  hal::callback<void()> reset;
  std::span<std::uint8_t> pcm8_buffer1;
  std::span<std::uint8_t> pcm8_buffer2;
};

// Application function must be implemented by one of the compilation units
// (.cpp) files.
void application(resource_list& p_map);
void initialize_platform(resource_list& p_resources);
