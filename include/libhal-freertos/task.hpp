#pragma once

#include <span>

#include <FreeRTOS.h>
#include <task.h>

#include <libhal/initializers.hpp>
#include <libhal/units.hpp>

namespace hal::freertos {
class static_task
{
public:
  using task_function = void(void*) noexcept;

  static_task(char const* p_name,
              task_function p_task_function,
              std::uint8_t p_priority,
              hal::buffer_param auto p_stack_size,
              std::size_t p_minimum_stack_size = 64)
    : static_task(p_name,
                  p_task_function,
                  hal::create_unique_static_buffer(p_stack_size),
                  p_priority,
                  p_minimum_stack_size)
  {
  }

  static_task(char const* p_name,
              task_function p_task_function,
              std::uint8_t p_priority,
              std::span<hal::byte> p_stack_buffer,
              std::size_t p_minimum_stack_size = 64);

  ~static_task();

private:
  StaticTask_t m_task_thread_control_block;
  TaskHandle_t m_handle;
};
}  // namespace hal::freertos
