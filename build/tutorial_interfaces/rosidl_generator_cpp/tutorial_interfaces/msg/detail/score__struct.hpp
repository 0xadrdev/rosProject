// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tutorial_interfaces:msg/Score.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__MSG__DETAIL__SCORE__STRUCT_HPP_
#define TUTORIAL_INTERFACES__MSG__DETAIL__SCORE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__tutorial_interfaces__msg__Score __attribute__((deprecated))
#else
# define DEPRECATED__tutorial_interfaces__msg__Score __declspec(deprecated)
#endif

namespace tutorial_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Score_
{
  using Type = Score_<ContainerAllocator>;

  explicit Score_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->first = 0l;
      this->second = 0l;
    }
  }

  explicit Score_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->first = 0l;
      this->second = 0l;
    }
  }

  // field types and members
  using _first_type =
    int32_t;
  _first_type first;
  using _second_type =
    int32_t;
  _second_type second;

  // setters for named parameter idiom
  Type & set__first(
    const int32_t & _arg)
  {
    this->first = _arg;
    return *this;
  }
  Type & set__second(
    const int32_t & _arg)
  {
    this->second = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tutorial_interfaces::msg::Score_<ContainerAllocator> *;
  using ConstRawPtr =
    const tutorial_interfaces::msg::Score_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tutorial_interfaces::msg::Score_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tutorial_interfaces::msg::Score_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tutorial_interfaces::msg::Score_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tutorial_interfaces::msg::Score_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tutorial_interfaces::msg::Score_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tutorial_interfaces::msg::Score_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tutorial_interfaces::msg::Score_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tutorial_interfaces::msg::Score_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tutorial_interfaces__msg__Score
    std::shared_ptr<tutorial_interfaces::msg::Score_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tutorial_interfaces__msg__Score
    std::shared_ptr<tutorial_interfaces::msg::Score_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Score_ & other) const
  {
    if (this->first != other.first) {
      return false;
    }
    if (this->second != other.second) {
      return false;
    }
    return true;
  }
  bool operator!=(const Score_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Score_

// alias to use template instance with default allocator
using Score =
  tutorial_interfaces::msg::Score_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tutorial_interfaces

#endif  // TUTORIAL_INTERFACES__MSG__DETAIL__SCORE__STRUCT_HPP_
