#pragma once

#if (__cpp_lib_unreachable >= 202202L)
#include <utility>
#endif

#include <stdexcept>
#include <string>
#include <string_view>
#include <fmt/core.h>
#include <source_location>

namespace candlewick {

[[noreturn]] inline void unreachable() {
#if (__cpp_lib_unreachable >= 202202L)
  std::unreachable();
#elif defined(_MSC_VER)
  __assume(false);
#else
  __builtin_unreachable();
#endif
}

/// \brief Wrapper for std::runtime_error, which prints out the filename and
/// code line.
struct RAIIException : std::runtime_error {
  RAIIException(std::string_view msg, std::source_location location =
                                          std::source_location::current());
};

namespace detail {

  std::string _error_message_impl(std::string_view fname,
                                  std::string_view fmtstr,
                                  fmt::format_args args);

  template <typename... Ts>
  std::string error_message_format(std::string_view fname,
                                   std::string_view _fmtstr, Ts &&...args) {
    return _error_message_impl(fname.data(), _fmtstr,
                               fmt::make_format_args(args...));
  }

} // namespace detail

// source_location default for last argument using ctad trick, see
// https://stackoverflow.com/a/71082768
template <typename... Ts> struct terminate_with_message {
  [[noreturn]] terminate_with_message(
      std::string_view fmt, Ts &&...args,
      std::source_location location = std::source_location::current()) {
    throw std::runtime_error(detail::error_message_format(
        location.function_name(), fmt, std::forward<Ts>(args)...));
  }

  [[noreturn]] terminate_with_message(std::source_location location,
                                      std::string_view fmt, Ts &&...args) {
    throw std::runtime_error(detail::error_message_format(
        location.function_name(), fmt, std::forward<Ts>(args)...));
  }
};

template <typename... Ts>
terminate_with_message(std::string_view, Ts &&...)
    -> terminate_with_message<Ts...>;

template <typename... Ts>
terminate_with_message(std::source_location, std::string_view, Ts &&...)
    -> terminate_with_message<Ts...>;

[[noreturn]]
void unreachable_with_message(
    std::string_view msg,
    std::source_location location = std::source_location::current());

} // namespace candlewick
