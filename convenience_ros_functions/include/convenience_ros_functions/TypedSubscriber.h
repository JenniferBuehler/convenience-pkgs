#ifndef CONVENIENCE_ROS_FUNCTIONS_TYPEDSUBSCRIBER_H
#define CONVENIENCE_ROS_FUNCTIONS_TYPEDSUBSCRIBER_H

#include <ros/ros.h>
#include <baselib_binding/Thread.h>

#include <map>

namespace convenience_ros_functions {

/**
 * Subscribes to a topic of a message type and provides the functionality to
 * wait for the next message.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
template<class MessageType>
class TypedSubscriber {
private:
    typedef TypedSubscriber<MessageType> Self;
public:
    /**
     */
    TypedSubscriber(ros::NodeHandle& _node):
        node(_node),
        running(false),
        subscriberActive(false),
        lastUpdateTime(0)
    {}

    ~TypedSubscriber(){}

    void start(const std::string& _topic);

    void stop();

    /**
     * Activates or deactivates the processing of incoming messages.
     * This can be used to save a bit of processing time when updates
     * are currently not required, but it is not desired to completely
     * stop the subscriber (with stop()) at times when updates are
     * not required.
     */
    void setActive(bool flag);

    /**
     * \return is active. This does not mean isRunning() also returns true:
     *      the service may have been stopped, but as soon as it is re-launched,
     *      the subscriber would automatically be active.
     */
    bool isActive() const;
    
    /**
     * \return if it is subscribed to a topic. May not necessarily be active,
     * use isActive() to check for this.
     */
    bool isRunning() const;

    /**
     * returns last arrived message in \e msg. 
     * \return false if no message has arrived yet.
     */
    bool getLastMessage(MessageType& msg) const;

    /**
     * Blocks while it waits for the incoming new message and returns it in \msg. 
     * Only succeeds if active (isActive() returns true)
     * \param timeout the timeout, or negative if no timeout to be used
     * \param wait_step time (in seconds) to sleep in-between checks wheter
     *      a new joint state has arrived.
     * \return true if successfully waited until next message or false
     *      if timeout reached (or isActive() returned false)
     */
    bool waitForNextMessage(MessageType& msg, float timeout = -1, float wait_step=0.05) const;

private:
    typedef baselib_binding::recursive_mutex recursive_mutex;
    typedef baselib_binding::mutex mutex;
    typedef baselib_binding::unique_lock<mutex>::type unique_lock;
    typedef baselib_binding::unique_lock<recursive_mutex>::type unique_recursive_lock;
    typedef baselib_binding::condition_variable condition_variable;

    ros::Time getLastUpdateTime() const;

    void msgCallback(const MessageType& _msg);

    // mutex to be used for all excepte messageArrivedCondition.
    // has to be locked *after* messageArrivedCondition *always*.
    mutable recursive_mutex generalMutex;
    
    mutable condition_variable messageArrivedCondition;
    // mutex to be used for messageArrivedCondition
    mutable mutex messageArrivedMutex;
    
    MessageType lastArrivedMessage;
    
    ros::Time lastUpdateTime;

    bool running;        
    bool subscriberActive;

    std::string topic;

    ros::NodeHandle& node;

    ros::Subscriber sub;
};

#include <convenience_ros_functions/TypedSubscriber.hpp>

}

#endif  // CONVENIENCE_ROS_FUNCTIONS_TYPEDSUBSCRIBER_H
