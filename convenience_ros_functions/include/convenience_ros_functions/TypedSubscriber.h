#ifndef CONVENIENCE_ROS_FUNCTIONS_TYPEDSUBSCRIBER_H
#define CONVENIENCE_ROS_FUNCTIONS_TYPEDSUBSCRIBER_H

#include <ros/ros.h>
#include <architecture_binding/Thread.h>

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
    /*{
        unique_recursive_lock lock(generalMutex);
        if (running && (topic==_topic)) return;
        if (running) stop();
        topic=_topic;
        sub= node.subscribe(topic, 1000, &Self::msgCallback,this);
        running=true;
    }*/

    void stop();
    /*{
        unique_recursive_lock lock(generalMutex);
        running=false;
        sub.shutdown();
    }*/

    /**
     * Activates or deactivates the processing of incoming messages.
     * This can be used to save a bit of processing time when updates
     * are currently not required, but it is not desired to completely
     * stop the subscriber (with stop()) at times when updates are
     * not required.
     */
    void setActive(bool flag);
    /*{
        unique_recursive_lock lock(generalMutex);
        subscriberActive = flag;
    }*/

    /**
     * \return is active. This does not mean isRunning() also returns true:
     *      the service may have been stopped, but as soon as it is re-launched,
     *      the subscriber would automatically be active.
     */
    bool isActive() const;
/*    {
        unique_recursive_lock lock(generalMutex);
        return subscriberActive;
    }
*/
    
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
  /*  {
        unique_recursive_lock lock(generalMutex);
        if (getLastUpdateTime() < 1e-03) return false;
        msg=lastArrivedMessage;
        return true;
    }*/


    /**
     * Blocks while it waits for the incoming new message and returns it in \msg. 
     * Only succeeds if active (isActive() returns true)
     * \param timeout the timeout, or negative if no timeout to be used
     * \param wait_step time (in seconds) to sleep in-between checks wheter
     *      a new joint state has arrived.
     * \return true if successfully waited until next message or false
     *      if timeout reached (or isActive() returned false)
     */
    bool waitForNextMessage(MessageType& msg, float timeout = -1, float wait_step=0.05) const
    /*{
        if (!isActive())
        {
            ROS_ERROR("Called TypedSubscriber::waitForNextMessage() without calling TypedSubscriber::setActive(true) first");
            return false;
        }

        ros::Time start_time = ros::Time::now();
        float time_waited=0;

        bool msgArrived=false;
        while (!msgArrived)
        {
            // ROS_INFO("TypedSubscriber: Waiting...");
            {
                unique_lock lock(messageArrivedMutex);
                // Unlocks the mutex and waits for a notification.
                msgArrived=this->messageArrivedCondition.timed_wait(lock, architecture_binding::get_duration_secs(wait_step));
            }
            if (!msgArrived)
            {   // it is still possible that a message arrived in-between the timed_wait calls, but we never
                // got the timing right for the condition trigger, especially if we use a low wait_step.
                // So do a separate check here.
                ros::Time last_upd = getLastUpdateTime();
                if (last_upd > start_time) msgArrived = true;
            }
            if (msgArrived)
            {
                // ROS_INFO("Message arrived!");
                unique_recursive_lock lock(generalMutex);
                msg=lastArrivedMessage;
                break;
            }
            // spin once so that msgCallback() may still be called by the subscriber
            // in case the node is not running in multi-threaded mode.
            ros::spinOnce();
            ros::Time curr_time = ros::Time::now();
            time_waited = (curr_time - start_time).toSec();
            if ((timeout > 0) && (time_waited > timeout))
            {   // timeout reached
                // ROS_INFO("Timeout reached");
                return false;
            }
        }
        return msgArrived;
    }*/

private:
    typedef architecture_binding::recursive_mutex recursive_mutex;
    typedef architecture_binding::mutex mutex;
    typedef architecture_binding::unique_lock<mutex>::type unique_lock;
    typedef architecture_binding::unique_lock<recursive_mutex>::type unique_recursive_lock;
    typedef architecture_binding::condition_variable condition_variable;

    ros::Time getLastUpdateTime() const;
/*    {
        unique_recursive_lock glock(generalMutex);
        return lastUpdateTime;
    }*/

    void msgCallback(const MessageType& _msg);
/*    {
        //ROS_INFO_STREAM("TYPED CALLBACK "<<_msg);
        unique_lock mlock(messageArrivedMutex);
        lastArrivedMessage=_msg;
        messageArrivedCondition.notify_all();
        unique_recursive_lock glock(generalMutex);
        lastUpdateTime = ros::Time::now();
    }
*/

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

}

#include <convenience_ros_functions/TypedSubscriber.hpp>

#endif  // CONVENIENCE_ROS_FUNCTIONS_TYPEDSUBSCRIBER_H
