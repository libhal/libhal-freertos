#pragma once

#include <array>
#include <span>

#include <libhal/initializers.hpp>
#include <libhal/units.hpp>

namespace hal::freertos {
class task
{
  using task_function = void(void*) noexcept;

  task(const char* p_name,
       task_function p_task_function,
       std::uint8_t p_priority,
       hal::buffer_param auto p_stack_size,
       std::size_t p_minimum_stack_size = 64)
    : task(hal::create_unique_static_buffer(p_stack_size),
           p_name,
           p_task_function,
           p_minimum_stack_size)
  {
  }

  task(const char* p_name,
       task_function p_task_function,
       std::uint8_t p_priority,
       std::span<hal::byte> p_stack_buffer,
       std::size_t p_minimum_stack_size = 64);

  ~task()
  {
  }

  StaticTask_t m_task_thread_control_block;
};
}  // namespace hal::freertos