#pragma once

#include "unreachable.h"

#include <SDL3/SDL_log.h>
#include <SDL3/SDL_assert.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <format>
#include <source_location>

namespace candlewick {

/// \brief Wrapper for std::runtime_error, which prints out the filename and
/// code line.
struct RAIIException : std::runtime_error {
  RAIIException(std::string_view msg, std::source_location location =
                                          std::source_location::current());
};

namespace detail {

  std::string _error_message_impl(const char *fname, std::string_view fmtstr,
                                  std::format_args args);

  template <typename... Ts>
  std::string error_message_format(std::string_view fname,
                                   std::string_view _fmtstr, Ts &&...args) {
    return _error_message_impl(
        fname.data(), _fmtstr,
        std::make_format_args(std::forward<Ts>(args)...));
  }

} // namespace detail

[[noreturn]]
inline void terminate_with_message(
    std::string_view msg,
    std::source_location location = std::source_location::current()) {
  throw std::runtime_error(
      detail::error_message_format(location.function_name(), "{:s}", msg)
          .c_str());
}

[[noreturn]]
inline void unreachable_with_message(
    std::string_view msg,
    std::source_location location = std::source_location::current()) {
  SDL_LogError(
      SDL_LOG_CATEGORY_APPLICATION, "%s",
      detail::error_message_format(location.function_name(), "{:s}", msg)
          .c_str());
  ::candlewick::unreachable();
}

} // namespace candlewick
