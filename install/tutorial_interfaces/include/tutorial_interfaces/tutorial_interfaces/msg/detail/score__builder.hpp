// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tutorial_interfaces:msg/Score.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__MSG__DETAIL__SCORE__BUILDER_HPP_
#define TUTORIAL_INTERFACES__MSG__DETAIL__SCORE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tutorial_interfaces/msg/detail/score__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tutorial_interfaces
{

namespace msg
{

namespace builder
{

class Init_Score_second
{
public:
  explicit Init_Score_second(::tutorial_interfaces::msg::Score & msg)
  : msg_(msg)
  {}
  ::tutorial_interfaces::msg::Score second(::tutorial_interfaces::msg::Score::_second_type arg)
  {
    msg_.second = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tutorial_interfaces::msg::Score msg_;
};

class Init_Score_first
{
public:
  Init_Score_first()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Score_second first(::tutorial_interfaces::msg::Score::_first_type arg)
  {
    msg_.first = std::move(arg);
    return Init_Score_second(msg_);
  }

private:
  ::tutorial_interfaces::msg::Score msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tutorial_interfaces::msg::Score>()
{
  return tutorial_interfaces::msg::builder::Init_Score_first();
}

}  // namespace tutorial_interfaces

#endif  // TUTORIAL_INTERFACES__MSG__DETAIL__SCORE__BUILDER_HPP_
