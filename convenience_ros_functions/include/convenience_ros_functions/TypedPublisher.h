#ifndef CONVENIENCE_ROS_FUNCTIONS_TYPEDSUBSCRIBER_H
#define CONVENIENCE_ROS_FUNCTIONS_TYPEDSUBSCRIBER_H

#include <ros/ros.h>
#include <architecture_binding/Thread.h>

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
/*    {
        unique_lock lock (mutex);    
        if (running && (topic==_topic)) return;
        if (running) stop();
        topic=_topic;
        pub= node.advertise<MessageType>(topic,queue_size,true);
        running=true;
    }
*/
    void stop();
/*    {
        unique_lock lock (mutex);    
        running=false;
        pub.shutdown();
    }
*/
    void publish(MessageType& m);
/*    {
        unique_lock lock (mutex);    
        if (!running) return;
        pub.publish(m);
    }
*/

    private:
    
    typedef architecture_binding::mutex mutex;
    typedef architecture_binding::unique_lock<mutex>::type unique_lock;


    mutex mutex;
    
    bool running;        

    std::string topic;
    ros::NodeHandle& node;

    ros::Publisher pub;
};

#include <convenience_ros_functions/TypedPublisher.hpp>

}

#endif   // CONVENIENCE_ROS_FUNCTIONS_TYPEDSUBSCRIBER_H
