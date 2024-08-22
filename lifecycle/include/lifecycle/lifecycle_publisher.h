#pragma once

#include <ros/ros.h>
#include <lifecycle/lifecycle_model.h>

namespace ros
{

namespace lifecycle
{

/**
 * @class LifecyclePublisher
 */
template<typename T>
class LifecyclePublisher {

 public:

  /** @brief Constructor */
 LifecyclePublisher( ros::NodeHandle _node_handle,
		     std::string _topic_name, uint32_t queue_size, bool latch = false) :
  node_handle_(_node_handle),
  state_(State::UNCONFIGURED),
  topic_name_(_topic_name),
  queue_size_(queue_size),
  latch_(latch)
 {}

  void on_configure()
  {
    pub_ = node_handle_.advertise<T>(topic_name_, queue_size_, latch_);
    state_ = State::INACTIVE;
  }

  void on_deactivate()
  {
    state_ = State::INACTIVE;
  }

  void on_activate()
  {
    state_ = State::ACTIVE;
  }

  void publish(T msg)
  {
    if(state_ == State::ACTIVE)
      pub_.publish(msg);
  }

  bool is_active()
  {
    return (state_ == State::ACTIVE);
  }

 protected:

  ros::lifecycle::State state_;
  ros::NodeHandle node_handle_;
  ros::Publisher pub_;

  std::string topic_name_;
  uint32_t queue_size_;
  bool latch_{false};
 };

} // namespace lifecycle

} // namespace ros
