template<typename ActionMessage>
ActionServer<ActionMessage>::ActionServer(
        ros::NodeHandle& _node, 
        const std::string& action_topic): 
    node(_node),
    initialized(false),
    hasGoal(false),
    actionTopic(action_topic),
    lastExeSuccess(false),
    actionServer(NULL),
    startTime(0), endTime (0)
{}

template<typename ActionMessage>
ActionServer<ActionMessage>::~ActionServer()
{
    this->deleteServer();    
}
   

template<typename ActionMessage>
bool ActionServer<ActionMessage>::init()
{
    startServer();
    bool success=initImpl();
    if (success) initialized = true;
    return success;
}

template<typename ActionMessage>
void ActionServer<ActionMessage>::shutdown()
{
    this->shutdownImpl();
    deleteServer();
    initialized = false;
}


template<typename ActionMessage>
bool ActionServer<ActionMessage>::executingGoal(){
    goalLock.lock();
    bool hasOneGoal=hasGoal;
    bool cancelled=false;
    if (hasOneGoal) {
        actionlib_msgs::GoalStatus stat=currentGoal.getGoalStatus();
        cancelled= (stat.status != actionlib_msgs::GoalStatus::ACTIVE);
            //(stat.status == actionlib_msgs::GoalStatus::PREEMPTED) 
            //|| (stat.status == actionlib_msgs::GoalStatus::ABORTED)
            //|| (stat.status == actionlib_msgs::GoalStatus::LOST);
    }
    goalLock.unlock();
    return ros::ok() && !cancelled && hasOneGoal;
}

template<typename ActionMessage>
float ActionServer<ActionMessage>::waitForExecution(float timeout){
    unique_lock guard(executionFinishedMutex);
    
    double exeTime=timeRunning(); //time the action already has been running
    
    ros::Time startWait=ros::Time::now();
    
    // Check if the action had already ended. This can
    // happen if he execution was so quick between starting it
    // and calling waitForExecution(), that it is ok to return the
    // last execution time. 
    if (!this->hasCurrentGoal()) return exeTime; 

    bool success=true;
    if (timeout < 0){
        // Unlocks the mutex and waits for a notification.
        this->executionFinishedCondition.wait(guard);
    }else{    
        // Unlocks the mutex and waits for a notification.
        success = COND_WAIT(this->executionFinishedCondition, guard, timeout);
    }
    if (!success) return -1;

    ros::Time endWait=ros::Time::now();
    float totalTime= exeTime+(endWait-startWait).toSec(); 
    return totalTime;
}

template<typename ActionMessage>
double ActionServer<ActionMessage>::timeRunning(){
    double ret=-1;
    if (this->hasCurrentGoal()){
        unique_lock lock(goalLock);
        ros::Time nowTime=ros::Time::now();
        ret=ros::Duration(nowTime-startTime).toSec();
    }else{
        unique_lock lock(goalLock);
        ret=ros::Duration(endTime-startTime).toSec();
    }
    return ret;
}

template<typename ActionMessage>
void ActionServer<ActionMessage>::currentActionDone(ResultT& result, const actionlib::SimpleClientGoalState& state){
    ROS_INFO_STREAM(this->actionTopic<<": Action finished. Result = "<<result);
    unique_lock guard( executionFinishedMutex );
    goalLock.lock();
    endTime=ros::Time::now();
    hasGoal = false;
    ROSFunctions::effectOnGoalHandle(state,currentGoal,result);
    lastExeSuccess=(state==actionlib::SimpleClientGoalState::SUCCEEDED);
    goalLock.unlock();
    executionFinishedCondition.notify_all();
}

template<typename ActionMessage>
void ActionServer<ActionMessage>::currentActionDone(const actionlib::SimpleClientGoalState& state)
{
    ROS_INFO_STREAM(this->actionTopic<<": Action finished.");
    unique_lock guard( executionFinishedMutex );
    goalLock.lock();
    endTime=ros::Time::now();
    hasGoal = false;
    ROSFunctions::effectOnGoalHandle(state,currentGoal);
    lastExeSuccess=(state==actionlib::SimpleClientGoalState::SUCCEEDED);
    goalLock.unlock();
    executionFinishedCondition.notify_all();
}

template<typename ActionMessage>
void ActionServer<ActionMessage>::currentActionSuccess(const bool success){
    ROS_INFO_STREAM(this->actionTopic<<": Action finished. Success = "<<success);
    if (success) currentActionDone(actionlib::SimpleClientGoalState::SUCCEEDED);
    else currentActionDone(actionlib::SimpleClientGoalState::ABORTED);
}

template<typename ActionMessage>
bool ActionServer<ActionMessage>::initImpl(){
    return true;
}

template<typename ActionMessage>
void ActionServer<ActionMessage>::startServer()
{
    if (actionServer)
    {
        delete actionServer;
    }
    actionServer = new ROSActionServerT(this->node,
        this->actionTopic,
        boost::bind(&Self::actionCallback, this, _1),
        boost::bind(&Self::actionCancelCallback, this, _1),
        false);
    actionServer->start();
}

template<typename ActionMessage>
void ActionServer<ActionMessage>::deleteServer(){
    if (this->actionServer) {
        delete actionServer;
        this->actionServer=NULL; //ROSActionServerPtr();
    }
}

template<typename ActionMessage>
#if ROS_VERSION_MINIMUM(1, 12, 0)
void ActionServer<ActionMessage>::actionCallback(ActionGoalHandleT goal)
#else
void ActionServer<ActionMessage>::actionCallback(ActionGoalHandleT& goal)
#endif
{
    ROS_INFO_STREAM(this->actionTopic<<": received new goal.");
    if (!this->initialized) {
        ROS_ERROR("Action server not initialised, can't accept goal");
        goal.setRejected();
        return;
    }

    if (this->hasCurrentGoal()){
        ROS_ERROR_STREAM(this->actionTopic<<": Goal currently running, can't accept this new goal");
        goal.setRejected();
        return;
    }

    // ROS_INFO("Checking whether goal can be accepted: ");
    if (!this->canAccept(goal))
    {
        ROS_ERROR_STREAM(this->actionTopic<<": Goal cannot be accepted");
        goal.setRejected();
        return;
    }

    ROS_INFO_STREAM(this->actionTopic<<": Goal accepted.");
    
    goalLock.lock();
    startTime=ros::Time::now();
    lastExeSuccess = false;
    hasGoal = true;
    currentGoal=goal;
    currentGoal.setAccepted();
    goalLock.unlock();
    
    this->actionCallbackImpl(goal);

}

template<typename ActionMessage>
#if ROS_VERSION_MINIMUM(1, 12, 0)
void ActionServer<ActionMessage>::actionCancelCallback(ActionGoalHandleT goal)
#else
void ActionServer<ActionMessage>::actionCancelCallback(ActionGoalHandleT& goal)
#endif
{
    this->actionCancelCallbackImpl(goal);
    currentActionDone(actionlib::SimpleClientGoalState::ABORTED);
}

template<typename ActionMessage>
bool ActionServer<ActionMessage>::hasCurrentGoal(){
    unique_lock lock(goalLock);
    return hasGoal;
}

