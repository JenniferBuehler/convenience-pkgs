#ifndef CONVENIENCE_ROS_FUNCTIONS_ACTIONSERVER_H
#define CONVENIENCE_ROS_FUNCTIONS_ACTIONSERVER_H

#include <architecture_binding/Thread.h>
#include <architecture_binding/SharedPtr.h>
#include <actionlib/server/action_server.h>
#include <convenience_ros_functions/ROSFunctions.h>

namespace convenience_ros_functions
{

/**
 * Helper class with an action server and usually required fields already defined.
 * Can be derived to provide a new action server.
 *
 * As a template parameter, pass the action message type.
 * For example, if there is a message type your_msgs::Example.action,
 * pass your_msgs::ExampleAction
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
template<class ActionMessage>
class ActionServer
{
protected:
    
    typedef ActionServer<ActionMessage> Self;

    typedef actionlib::ActionServer<ActionMessage> ROSActionServerT;
    // typedef typename architecture_binding::shared_ptr<ROSActionServerT> ROSActionServerPtr;
    typedef ROSActionServerT * ROSActionServerPtr;
    typedef typename ROSActionServerT::GoalHandle ActionGoalHandle;
    typedef typename ActionMessage::_action_result_type ActionResult;
    typedef typename ActionMessage::_action_goal_type ActionGoal;
    typedef typename ActionMessage::_action_feedback_type ActionFeedback;

public:
    ActionServer<ActionMessage>(
            ros::NodeHandle& _node, 
            const std::string& action_topic): 
        node(_node),
        initialized(false),
        hasGoal(false),
        actionTopic(action_topic),
        lastExeSuccess(false),
        actionServer(NULL),
        startTime(0), endTime (0)
    {
    }

    virtual ~ActionServer<ActionMessage>(){
        this->deleteServer();    
    }
       
 
	/**
	 * Starts action server and does internal initialisation.
	 */
    bool init()
    {
        startServer();
        bool success=initImpl();
        if (success) initialized = true;
        return success;
    }

	/**
 	 * Methods to be executed for shutting down the action server
	 */
    void shutdown()
    {
        this->shutdownImpl();
        deleteServer();
        initialized = false;
    }


    /**
     * Checks whether there is currently a goal, and it is also active.
     */
    bool executingGoal(){
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

protected:

    /**
     * Can be used by subclasses to wait for the current execution of the action.
     * Internally, waits until currentActionDone() is called by subclasses.
     * \return the duration of the wait. If the action has already finished
     *      (is not running), returns the duration of the last executed action.
     */
    virtual float waitForExecution(float timeout){
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
            success = this->executionFinishedCondition.timed_wait(guard, architecture_binding::get_duration_secs(timeout));
        }
        if (!success) return -1;

        ros::Time endWait=ros::Time::now();
        float totalTime= exeTime+(endWait-startWait).toSec(); 
        return totalTime;
    }


    //Time (in seconds) which has passed since the action has been started.
    double timeRunning(){
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


    /**
     * Method to call from subclasses to signal that the currently running goal has finished.
     * Alternatively, other methods called currentActionDone() can be called which have the 
     * same effect but less descriptive outcomes for the action result.
     */
    void currentActionDone(ActionResult& result, const actionlib::SimpleClientGoalState& state){
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

    /**
     * Simpler implementation of currentActionDone(ActionResult&, const actionlib::SimpleClientGoalState&)
     * which does not take a result.
     */
    void currentActionDone(const actionlib::SimpleClientGoalState& state){
        ROS_INFO_STREAM(this->actionTopic<<": Action finished. SimpleClientGoalState = "<<state);
        unique_lock guard( executionFinishedMutex );
        goalLock.lock();
        endTime=ros::Time::now();
        hasGoal = false;
        ROSFunctions::effectOnGoalHandle(state,currentGoal);
        lastExeSuccess=(state==actionlib::SimpleClientGoalState::SUCCEEDED);
        goalLock.unlock();
        executionFinishedCondition.notify_all();
    }
   
    /**
     * Simple implementation of other currentActionDone() methods but which has same
     * effect. When successful, it works the same. On error, the goal is always
     * set to "aborted".
     * The only reason this action is called "currentActionSuccess" instead of
     * "currentActionDone" is that while it compiles, at runtime there is the problem
     * that it recursively calls itself, instead of calling
     * currentActionDone(const actionlib::SimpleClientGoalState).
     */ 
    void currentActionSuccess(const bool success){
        ROS_INFO_STREAM(this->actionTopic<<": Action finished. Success = "<<success);
        if (success) currentActionDone(actionlib::SimpleClientGoalState::SUCCEEDED);
        else currentActionDone(actionlib::SimpleClientGoalState::ABORTED);
    }

    /**
     * Can be implemented by subclasses. This is called from init().
     */
    virtual bool initImpl(){
        return true;
    }
    
    /**
     * Can be implemented by subclasses. This is called from shutdown().
     */
    virtual void shutdownImpl() {}
 
    /**
     * Subclasses should implement here whether this goal is eligible.
     * No need to set goal to any state, this will be done if
     * this method returns false. Will *always* be called immediately
     * before actionCallbackImpl().
     */
    virtual bool canAccept(const ActionGoalHandle& goal)=0;

    /**
     * Receive a new goal: subclasses implementation. No need to set the goal to accepted, rejected, etc.
     * This function should just be used to initialize interal fields based on the values of the
     * goal handle, and to kick off the execution of the action. Method canAccept() should have been
     * used before to rule out that this action can be started.
     *
     * Once the action is done, call method currentActionDone() to finalize the execution of
     * this goal.
     */
    virtual void actionCallbackImpl(const ActionGoalHandle& goal)=0;

    /**
     * Receive a cancel instruction: subclasses implementation.
     * Only needs to be implemented if any special actions are required
     * upon canceling the action. 
     */
    virtual void actionCancelCallbackImpl(ActionGoalHandle& goal) {}

private:

    void startServer()
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

    void deleteServer(){
        if (this->actionServer) {
            delete actionServer;
            this->actionServer=NULL; //ROSActionServerPtr();
        }
    }

    /**
     * Receive a new goal
     */
    void actionCallback(ActionGoalHandle& goal){
        ROS_INFO("ActionServer: received new goal.");
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

    /**
     * Receive a cancel instruction
     */
    void actionCancelCallback(ActionGoalHandle& goal){
        this->actionCancelCallbackImpl(goal);
        currentActionDone(actionlib::SimpleClientGoalState::ABORTED);
    }

    bool hasCurrentGoal(){
        unique_lock lock(goalLock);
        return hasGoal;
    }


    typedef architecture_binding::recursive_mutex recursive_mutex;
    typedef architecture_binding::mutex mutex;
    typedef architecture_binding::unique_lock<mutex>::type unique_lock;
    typedef architecture_binding::unique_lock<recursive_mutex>::type unique_recursive_lock;
    typedef architecture_binding::condition_variable condition_variable;


    ros::NodeHandle node;

    std::string actionTopic;
    
    bool initialized;
    bool hasGoal;
    bool lastExeSuccess;
    ActionGoalHandle currentGoal;
    mutex goalLock; //to lock access to currentGoal

    ROSActionServerPtr actionServer;
    ros::Time startTime, endTime;
    condition_variable executionFinishedCondition;
    mutex executionFinishedMutex;
};


}  // namespace
#endif   // CONVENIENCE_ROS_FUNCTIONS_ACTIONSERVER_H
