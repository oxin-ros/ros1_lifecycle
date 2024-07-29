//
// ROS1 Lifecycle - A library implementing the ROS2 lifecycle for ROS1
//
// Copyright 2016,2017 Robert Bosch GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include <memory>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <lifecycle/lifecycle_model.h>
#include <lifecycle/lifecycle_publisher.h>
#include <lifecycle/managed_node.h>

typedef ros::lifecycle::LifecyclePublisher<std_msgs::String> StringPublisher;

class ExampleManagedNode : public ros::lifecycle::ManagedNode {
public:
    //The node handle has to be private namespace, for all the lifecycle related topics to be unambiguous.
    ExampleManagedNode(ros::NodeHandle& nh) : _nh(nh), ros::lifecycle::ManagedNode(nh) {
    }

protected:
    // override must-have functions from managed node
    bool onActivate() {
        ROS_INFO("Activating");

        _pub->on_activate();
        _timer->start();

        return true;
    };

    // define other virtual methods
    bool onConfigure() {
        ROS_INFO("Configuring");

        _pub = std::make_shared<StringPublisher>(shared_from_this(), "chatter", 10, false);
        _pub->on_configure();
        _timer = std::make_shared<ros::Timer>(_nh.createTimer(ros::Duration(1.0), &ExampleManagedNode::timerCallback, this));
        _timer->stop();
        _sub = std::make_shared<ros::Subscriber>(_nh.subscribe("noise", 10, &ExampleManagedNode::callback, this));
        _server = std::make_shared<ros::ServiceServer>(_nh.advertiseService("example_service", &ExampleManagedNode::serverCallback, this));

        ROS_INFO("Configured");
        return true;
    };

    bool onDeactivate() {
        ROS_INFO("Deactivating");

        _pub->on_deactivate();
        _timer->stop();
        _sub->shutdown();
        _server->shutdown();

        return true;
    };

    bool onShutdown() {
        ROS_INFO("Shutting down");

        _pub = nullptr;
        _timer = nullptr;
        _sub = nullptr;
        _server = nullptr;

        return true;
    };

    bool onCleanup() {
        ROS_INFO("Cleaning up");

        _pub.reset();
        _sub.reset();
        _timer.reset();
        _server.reset();

        return true;
    };

private:
    void timerCallback(const ros::TimerEvent& event) {
        std_msgs::String msg;
        msg.data = "Hello, world!";
        _pub->publish(msg);
    }

    void callback(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("I heard: [%s]", msg->data.c_str());
    }

    bool serverCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        if (getCurrentState() != ros::lifecycle::State::ACTIVE)
            return false;
        res.success = true;
        res.message = "Success!";
        return true;
    }

    ros::NodeHandle _nh;
    std::shared_ptr<StringPublisher> _pub{nullptr};
    std::shared_ptr<ros::Subscriber> _sub{nullptr};
    std::shared_ptr<ros::Timer> _timer{nullptr};
    std::shared_ptr<ros::ServiceServer> _server{nullptr};
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "example_lcm_node");
    ros::NodeHandle nh("~");
    // If this is true, the node will transition to the active state even without
    // the presence of a Manager.
    nh.setParam(PARAM_LIFECYCLE_MANAGEMENT, true);

    ExampleManagedNode node(nh);

    ros::spin();
}