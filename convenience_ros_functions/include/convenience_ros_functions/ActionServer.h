#ifndef CONVENIENCE_ROS_FUNCTIONS_ACTIONSERVER_H
#define CONVENIENCE_ROS_FUNCTIONS_ACTIONSERVER_H

#include <baselib_binding/Thread.h>
#include <baselib_binding/SharedPtr.h>
#include <actionlib/server/action_server.h>
#include <convenience_ros_functions/ROSFunctions.h>

namespace convenience_ros_functions
{

/**
 * \brief Helper class with an action server and commonly required fields already defined.
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
    ACTION_DEFINITION(ActionMessage)
protected:
    typedef ActionServer<ActionMessage> Self;

    typedef actionlib::ActionServer<ActionMessage> ROSActionServerT;
    // typedef typename baselib_binding::shared_ptr<ROSActionServerT> ROSActionServerPtr;
    typedef ROSActionServerT * ROSActionServerPtr;
    typedef typename ROSActionServerT::GoalHandle ActionGoalHandleT;

    typedef Goal GoalT;
    typedef GoalConstPtr GoalConstPtrT;
    typedef ActionGoal ActionGoalT;
    typedef ActionGoalPtr ActionGoalPtrT;
    typedef ActionGoalConstPtr ActionGoalConstPtrT;
    typedef ActionResult ActionResultT;
    typedef ActionResultConstPtr ActionResultConstPtrT;
    typedef Result ResultT;
    typedef ResultConstPtr ResultConstPtrT;
    typedef Feedback FeedbackT;
    typedef FeedbackConstPtr FeedbackConstPtrT;
    typedef ActionFeedback ActionFeedbackT;
    typedef ActionFeedbackConstPtr ActionFeedbackConstPtrT;
public:

    ActionServer<ActionMessage>(
            ros::NodeHandle& _node,
            const std::string& action_topic);
    virtual ~ActionServer<ActionMessage>();


	/**
	 * Starts action server and does internal initialisation.
	 */
    bool init();

	/**
 	 * Methods to be executed for shutting down the action server
	 */
    void shutdown();

    /**
     * Checks whether there is currently a goal, and it is also active.
     */
    bool executingGoal();

protected:

    /**
     * Can be used by subclasses to wait for the current execution of the action.
     * Internally, waits until currentActionDone() is called by subclasses.
     * \return the duration of the wait. If the action has already finished
     *      (is not running), returns the duration of the last executed action.
     */
    virtual float waitForExecution(float timeout);

    /**
     * Time (in seconds) which has passed since the action has been started.
     */
    double timeRunning();

    /**
     * Method to call from subclasses to signal that the currently running goal has finished.
     * Alternatively, other methods called currentActionDone() can be called which have the
     * same effect but less descriptive outcomes for the action result.
     */
    void currentActionDone(ResultT& result, const actionlib::SimpleClientGoalState& state);

    /**
     * Simpler implementation of currentActionDone(ActionResult&, const actionlib::SimpleClientGoalState&)
     * which does not take a result.
     */
    void currentActionDone(const actionlib::SimpleClientGoalState& state);

    /**
     * Simple implementation of other currentActionDone() methods but which has same
     * effect. When successful, it works the same. On error, the goal is always
     * set to "aborted".
     * The only reason this action is called "currentActionSuccess" instead of
     * "currentActionDone" is that while it compiles, at runtime there is the problem
     * that it recursively calls itself, instead of calling
     * currentActionDone(const actionlib::SimpleClientGoalState).
     */
    void currentActionSuccess(const bool success);

    /**
     * Can be implemented by subclasses. This is called from init().
     */
    virtual bool initImpl();

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
    virtual bool canAccept(const ActionGoalHandleT& goal)=0;

    /**
     * Receive a new goal: subclasses implementation. No need to set the goal to accepted, rejected, etc.
     * This function should just be used to initialize interal fields based on the values of the
     * goal handle, and to kick off the execution of the action. Method canAccept() should have been
     * used before to rule out that this action can be started.
     *
     * Once the action is done, call method currentActionDone() to finalize the execution of
     * this goal.
     */
    virtual void actionCallbackImpl(const ActionGoalHandleT& goal)=0;

    /**
     * Receive a cancel instruction: subclasses implementation.
     * Only needs to be implemented if any special actions are required
     * upon canceling the action.
     */
    virtual void actionCancelCallbackImpl(const ActionGoalHandleT& goal) {}

private:

    void startServer();

    void deleteServer();

#if ROS_VERSION_MINIMUM(1, 12, 0)
    /**
     * Receive a new goal
     */
    void actionCallback(ActionGoalHandleT goal);

    /**
     * Receive a cancel instruction
     */
    void actionCancelCallback(ActionGoalHandleT goal);
#else
    /**
     * Receive a new goal
     */
    void actionCallback(ActionGoalHandleT& goal);

    /**
     * Receive a cancel instruction
     */
    void actionCancelCallback(ActionGoalHandleT& goal);
#endif

    bool hasCurrentGoal();

    typedef baselib_binding::recursive_mutex recursive_mutex;
    typedef baselib_binding::mutex mutex;
    typedef baselib_binding::unique_lock<mutex>::type unique_lock;
    typedef baselib_binding::unique_lock<recursive_mutex>::type unique_recursive_lock;
    typedef baselib_binding::condition_variable condition_variable;


    ros::NodeHandle node;

    std::string actionTopic;

    bool initialized;
    bool hasGoal;
    bool lastExeSuccess;
    ActionGoalHandleT currentGoal;
    mutex goalLock; //to lock access to currentGoal

    ROSActionServerPtr actionServer;
    ros::Time startTime, endTime;
    condition_variable executionFinishedCondition;
    mutex executionFinishedMutex;
};

#include <convenience_ros_functions/ActionServer.hpp>

}  // namespace
#endif   // CONVENIENCE_ROS_FUNCTIONS_ACTIONSERVER_H
