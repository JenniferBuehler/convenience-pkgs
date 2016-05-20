#ifndef CONVENIENCE_ROS_FUNCTIONS_TYPEDSUBSCRIBER_H
#define CONVENIENCE_ROS_FUNCTIONS_TYPEDSUBSCRIBER_H

#include <ros/ros.h>
#include <baselib_binding/Thread.h>

namespace convenience_ros_functions {

/**
 * Simple helper class to publish messages of a type on a topic.
 * 
 * \author Jennifer Buehler
 * \date March 2016
 */
template<class MessageType>
class TypedPublisher {

    public:
    TypedPublisher(ros::NodeHandle& _node):
        node(_node),
        running(false)
    {}

    ~TypedPublisher(){}

    void start(const std::string& _topic, int queue_size=100);

    void stop();

    void publish(MessageType& m);

    private:
    
    typedef baselib_binding::mutex mutex;
    typedef baselib_binding::unique_lock<mutex>::type unique_lock;

    mutex mutex;
    
    bool running;        

    std::string topic;
    ros::NodeHandle& node;

    ros::Publisher pub;
};

#include <convenience_ros_functions/TypedPublisher.hpp>

}

#endif   // CONVENIENCE_ROS_FUNCTIONS_TYPEDSUBSCRIBER_H
