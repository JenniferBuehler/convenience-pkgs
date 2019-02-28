template<typename Msg>
void TypedSubscriber<Msg>::start(const std::string& _topic){
    unique_recursive_lock lock(generalMutex);
    if (running && (topic==_topic)) return;
    if (running) stop();
    topic=_topic;
    sub= node.subscribe(topic, 1000, &Self::msgCallback,this);
    running=true;
}

template<typename Msg>
void TypedSubscriber<Msg>::stop(){
    unique_recursive_lock lock(generalMutex);
    if (!running) return;
    running=false;
    sub.shutdown();
}

template<typename Msg>
void TypedSubscriber<Msg>::setActive(bool flag)
{
    unique_recursive_lock lock(generalMutex);
    if (subscriberActive == flag) return;
    if (flag && !running)
    {
        ROS_ERROR("Cannot activate TypedSubscriber if it is not running");
        return;
    }
    subscriberActive = flag;
}

template<typename Msg>
bool TypedSubscriber<Msg>::isActive() const
{
    unique_recursive_lock lock(generalMutex);
    return subscriberActive;
}

template<typename Msg>
bool TypedSubscriber<Msg>::isRunning() const
{
    unique_recursive_lock lock(generalMutex);
    return running;
}

template<typename Msg>
bool TypedSubscriber<Msg>::getLastMessage(Msg& msg) const
{
    unique_recursive_lock lock(generalMutex);
    if (getLastUpdateTime() < 1e-03) return false;
    msg=lastArrivedMessage;
    return true;
}

template<typename Msg>
bool TypedSubscriber<Msg>::waitForNextMessage(Msg& msg, float timeout, float wait_step) const
{
    if (!isActive())
    {
        ROS_ERROR("Called TypedSubscriber<Msg>::waitForNextMessage() without calling TypedSubscriber<Msg>::setActive(true) first");
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
            // msgArrived=this->messageArrivedCondition.timed_wait(lock, baselib_binding::get_duration_secs(wait_step));
            msgArrived = COND_WAIT(this->messageArrivedCondition, lock, wait_step);
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
}

template<typename Msg>
ros::Time TypedSubscriber<Msg>::getLastUpdateTime() const
{
    unique_recursive_lock glock(generalMutex);
    return lastUpdateTime;
}

template<typename Msg>
void TypedSubscriber<Msg>::msgCallback(const Msg& _msg)
{
    //ROS_INFO_STREAM("TYPED CALLBACK "<<_msg);
    unique_lock mlock(messageArrivedMutex);
    lastArrivedMessage=_msg;
    messageArrivedCondition.notify_all();
    unique_recursive_lock glock(generalMutex);
    lastUpdateTime = ros::Time::now();
}

