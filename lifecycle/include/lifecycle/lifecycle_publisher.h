#pragma once

#include <ros/ros.h>
#include <lifecycle/lifecycle_model.h>
#include <lifecycle/managed_node.h>

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
 LifecyclePublisher( std::shared_ptr<ManagedNode> _node,
		     std::string _topic_name, uint32_t queue_size, bool latch = false) :
  node_(_node),
  state_(State::UNCONFIGURED),
  topic_name_(_topic_name),
  queue_size_(queue_size),
  latch_(latch)
 {}

  void on_configure()
  {
    pub_ = node_->getBaseNode().advertise<T>(topic_name_, queue_size_, latch_);
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
  ros::Publisher pub_;

  std::shared_ptr<ManagedNode> node_;
  std::string topic_name_;
  uint32_t queue_size_;
  bool latch_{false};
 };

} // namespace lifecycle
 
} // namespace ros
