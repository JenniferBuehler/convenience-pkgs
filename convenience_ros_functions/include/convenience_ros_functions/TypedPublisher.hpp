namespace convenience_ros_functions
{

template<typename Msg>
void TypedPublisher::start(const std::string& _topic, int queue_size=100)
{
    unique_lock lock (mutex);    
    if (running && (topic==_topic)) return;
    if (running) stop();
    topic=_topic;
    pub= node.advertise<MessageType>(topic,queue_size,true);
    running=true;
}

template<typename Msg>
void TypedPublisher::stop(){
    unique_lock lock (mutex);    
    running=false;
    pub.shutdown();
}

template<typename Msg>
void TypedPublisher::publish(MessageType& m){
    unique_lock lock (mutex);    
    if (!running) return;
    pub.publish(m);
}

}  // namespace
