// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tutorial_interfaces:msg/Score.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__MSG__DETAIL__SCORE__TRAITS_HPP_
#define TUTORIAL_INTERFACES__MSG__DETAIL__SCORE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tutorial_interfaces/msg/detail/score__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tutorial_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Score & msg,
  std::ostream & out)
{
  out << "{";
  // member: first
  {
    out << "first: ";
    rosidl_generator_traits::value_to_yaml(msg.first, out);
    out << ", ";
  }

  // member: second
  {
    out << "second: ";
    rosidl_generator_traits::value_to_yaml(msg.second, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Score & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: first
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first: ";
    rosidl_generator_traits::value_to_yaml(msg.first, out);
    out << "\n";
  }

  // member: second
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second: ";
    rosidl_generator_traits::value_to_yaml(msg.second, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Score & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace tutorial_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use tutorial_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tutorial_interfaces::msg::Score & msg,
  std::ostream & out, size_t indentation = 0)
{
  tutorial_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tutorial_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const tutorial_interfaces::msg::Score & msg)
{
  return tutorial_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tutorial_interfaces::msg::Score>()
{
  return "tutorial_interfaces::msg::Score";
}

template<>
inline const char * name<tutorial_interfaces::msg::Score>()
{
  return "tutorial_interfaces/msg/Score";
}

template<>
struct has_fixed_size<tutorial_interfaces::msg::Score>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<tutorial_interfaces::msg::Score>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<tutorial_interfaces::msg::Score>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TUTORIAL_INTERFACES__MSG__DETAIL__SCORE__TRAITS_HPP_
