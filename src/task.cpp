#include <cstdint>

#include <algorithm>
#include <span>

#include <libhal-freertos/task.hpp>

#include <libhal/error.hpp>

namespace hal::freertos {
constexpr std::uint8_t tls_index = 0;
extern "C"
{
  extern std::uint32_t __tls_base;
  extern std::uint32_t __tdata_source;
  extern std::uint32_t __tdata_size;
  extern std::uint32_t __tls_size;

  void* __emutls_get_address(unsigned p_offset)
  {
    auto* tls_memory = pvTaskGetThreadLocalStoragePointer(nullptr, tls_index);
    auto* final_address = tls_memory + p_offset;

    return final_address;
  }
}

task::task(const char* p_name,
           task_function p_task_function,
           std::uint8_t p_priority,
           std::span<hal::byte> p_stack_buffer,
           std::size_t p_minimum_stack_size)
{
  // Setup TLS Section
  const auto tls_size = reinterpret_cast<std::uintptr_t>(&__tls_size);
  const auto tls_data_size = reinterpret_cast<std::uintptr_t>(&__tdata_size);

  // If the tls_size is greater than or equal to the
  if (tls_size >= p_stack_buffer.size() + p_minimum_stack_size) {
    hal::safe_throw(hal::argument_out_of_domain(this));
  }

  // Use the top of the stack buffer for TLS memory
  auto tls_buffer = p_stack_buffer.first(tls_size);

  // Copy the contents of the tls data section from its source (flash) to the
  // tls buffer.
  memcpy(tls_buffer.data(), &__tdata_source, tls_size);
  memset(tls_buffer.data() + tls_size, 0, tls_size);  // TODO fix this

  auto stack_buffer = p_stack_buffer.subspan(tls_size);

  auto handle = xTaskCreateStatic(p_task_function,
                                  p_name,
                                  stack_buffer.size(),
                                  nullptr,
                                  p_priority,
                                  stack_buffer.data(),
                                  &m_task_thread_control_block);

  vTaskSetThreadLocalStoragePointer(handle, tls_index, tls_buffer.data());
}
}  // namespace hal::freertos
