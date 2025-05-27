#include <FreeRTOS.h>
#include <portmacro.h>
#include <task.h>

#include <cstdint>
#include <cstring>
#include <span>

#include <libhal-freertos/task.hpp>
#include <libhal/error.hpp>

#include "tls_support.hpp"

namespace hal::freertos {
constexpr std::uint8_t tls_index = 0;

static_task::static_task(char const* p_name,
                         task_function p_task_function,
                         std::uint8_t p_priority,
                         std::span<hal::byte> p_stack_buffer,
                         std::size_t p_minimum_stack_size)
{
  // Setup TLS Section
  auto const tls_size = reinterpret_cast<std::uintptr_t>(&__tls_size);
  auto const tls_data_size = reinterpret_cast<std::uintptr_t>(&__tdata_size);

  // If the tls_size is greater than or equal to the
  if (tls_size >= p_stack_buffer.size() + p_minimum_stack_size) {
    hal::safe_throw(hal::argument_out_of_domain(this));
  }

  // Use the top of the stack buffer for TLS memory
  auto tls_buffer = p_stack_buffer.first(tls_size);

  // Copy the contents of the tls data section from its source (flash) to the
  // tls buffer.
  memcpy(tls_buffer.data(), &__tdata_source, tls_data_size);
  memset(tls_buffer.data() + tls_data_size, 0, tls_size - tls_data_size);

  auto stack_buffer_remaining = p_stack_buffer.subspan(tls_size);
  std::span<StackType_t> stack_buffer{
    reinterpret_cast<StackType_t*>(stack_buffer_remaining.data()),
    stack_buffer_remaining.size() / sizeof(StackType_t)
  };

  m_handle = xTaskCreateStatic(p_task_function,
                               p_name,
                               stack_buffer.size(),
                               nullptr,
                               p_priority,
                               stack_buffer.data(),
                               &m_task_thread_control_block);

  vTaskSetThreadLocalStoragePointer(m_handle, tls_index, tls_buffer.data());
}

static_task::~static_task()
{
  vTaskDelete(m_handle);
}
}  // namespace hal::freertos
