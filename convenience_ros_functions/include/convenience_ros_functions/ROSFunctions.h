#ifndef CONVENIENCE_ROS_FUNCTIONS_ROSFUNCTIONS_H
#define CONVENIENCE_ROS_FUNCTIONS_ROSFUNCTIONS_H

#include <baselib_binding/SharedPtr.h>
#include <baselib_binding/Thread.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <fstream>

#include <boost/foreach.hpp>
// XXX TODO: get rid of direct boost dependency

#define DEFAULT_TF_CACHE_TIME 15

namespace convenience_ros_functions
{

/**
 * Keeps a local tf listener which can be shared via a static singleton. It is important, if you
 * use this singleton, to call initSingleton(), or obtain a reference to it via Singleton(),
 * when you want the tf listener to be created and start listening. Once initSingleton()/Singleton()
 * has been called, the tf listener will keep listening, until the program ends of destroySingleton() is called.
 * It can be that the tf listener needs some time to initialize, so it's advisable to
 * call initSingleton() before you need it for the first time in a time-critical context.
 *
 * \author Jennifer Buehler
 */
class ROSFunctions
{
public:
    typedef baselib_binding::shared_ptr<ROSFunctions>::type ROSFunctionsPtr;
    /**
     * \param tf_max_cache_time maximum cache time for tf listener
     */
    ROSFunctions(float tf_max_cache_time = DEFAULT_TF_CACHE_TIME);

    ~ROSFunctions()
    {
    }

    /**
     * Initializes the singleton. If this function already was called, it does nothing.
     * Otherwise, it initializes the singleton, which will incur a small wait to ensure the tf listener
     * has received its first frame. If a transform function is called immediately,
     * there are sometimes problems with it.
     * This MUST be called from a valid ROS node!
     */
    static void initSingleton();
    static void destroySingleton();

    /**
     * Returns the singleton. If initSingleton() has not been called before,
     * it is called from here. However it is recommended to explicitly call initSingleton()
     * in advance. See also comment in initSingleton(). This MUST be called from a valid
     * ROS node!
     */
    static ROSFunctionsPtr Singleton();

    template<typename ROSMessage>
    static bool saveToFile(const ROSMessage& msg, const std::string& filename, bool asBinary);

    template<typename ROSMessage>
    static bool readFromFile(const std::string& filename, ROSMessage& msg, bool isBinary);

    template<typename ROSMessage>
    static bool saveToBagFile(const std::vector<ROSMessage>& msgs, const std::string& filename);

    template<typename ROSMessage>
    static bool readFromBagFile(const std::string& filename, std::vector<ROSMessage>& msgs);

    /**
     * Takes \e joint_state and re-assigns all its fields with the values
     * in target_joints, if the same joint appears in there.
     * All values in target_joints which are not in joint_state, are added to it.
     */
    static void assignJointState(const sensor_msgs::JointState& target_joints,
                                 sensor_msgs::JointState& joint_state);


    /**
     * Creates a new joint state with only the joint names which are both in s1 and s2.
     * The current values of the joints are initialized with the ones set s1 if parameter
     * init_s1 is true, otherwise with values in s2. The order the joint names appear are as in s1.
     *
     * We may impose the restriction that ALL values in s1 also HAVE TO appear in
     * s2 (s2 is subset of s1), by setting the parameter s2_is_subset to true. In this case, the function
     * will return an error if s2 is not subset of s1. This can be used to check consistency.
     */
    static bool intersectJointState(const sensor_msgs::JointState& s1, const sensor_msgs::JointState& s2,
                                    sensor_msgs::JointState& result,
                                    bool init_s1, bool s2_is_subset);

    /**
     * Works like doing s1=s2, but only copying the values in s2 that also appear in s1.
     * We may impose the restriction that ALL values in s1 also HAVE TO appear in s2 (s2 is subset of s1),
     * by setting the parameter s2_is_subset to true. In this case, the function will
     * return an error if s2 is not subset of s1. This can be used to check consistency.
     */
    static bool intersectJointStates(const sensor_msgs::JointState& s1,
                                     const sensor_msgs::JointState& s2,
                                    sensor_msgs::JointState& result, bool s2_is_subset);


    /**
     * Simple implementation to compare joint states, similar to equalJointPositions(), but
     * more efficient. Instead, some assumptions are made:
     * It assumes a common subset of joints are listed (in same order)
     * at the beginning of both states. One state may be larger than the other.
     */
    static bool equalJointPositionsSimple(const sensor_msgs::JointState& j1, const sensor_msgs::JointState& j2,
                                const float pos_tolerance);
   
    /**
     * Compares joint states: All joints appearing in *both* joint states are required to be
     * as similar as \e pos_tolerance
     * \retval 1 requirements satisfied
     * \retval -1 requirements not satisfied
     * \retval -2 there is no complete intersection of both joint states
     * \retval <-2 other consistency error 
     */ 
    static int equalJointPositions(const sensor_msgs::JointState& j1, const sensor_msgs::JointState& j2,
                                const float pos_tolerance);


    static bool getJointStateAt(int idx, const trajectory_msgs::JointTrajectory& traj, sensor_msgs::JointState& result);


    /**
     * MJointState must be such as trajectory_msgs::MultiDOFJointTrajectoryPoint or
     * sensor_msgs::MultiDOFJointState, such that <type>.transforms is a vector of transforms of
     * same size as joint_names, containing a transform for each joint.
     */
    template<typename MJointState>
    static bool getPoseFromVirtualJointState(const std::vector<std::string>& joint_names,
            const MJointState& virtualJointState,
            const std::string& virtualJointName, geometry_msgs::PoseStamped& robotPose);

    /**
     * Checks whether based on the current tf transforms a distance between the two poses can be obtained.
     * \param latest The lastest time of both stamps is assumed, or otherwise the most recent transform available for both
     * \param p1 target pose
     * \param p2 source pose
     */
    bool canGetTransform(const std_msgs::Header& p1, const std_msgs::Header& p2, bool latest, bool printError) const;
    /**
     * \param f1 target frame
     * \param f2 source frame
     */
    bool canGetTransform(const std::string& f1, const std::string& f2, const ros::Time& useTime, bool printError) const;

    /**
     * calls tf_listener::waitForTransform to see if distance between two poses can be obtained.
     * \param latest The lastest time of both stamps is assumed, or otherwise the most recent transform available for both
     * \param p1 target pose
     * \param p2 source pose
     */
    bool waitForTransform(const std_msgs::Header& p1, const std_msgs::Header& p2,
                          const float& timeout, bool latest, bool printError);
    /**
     * \param f1 target frame
     * \param f2 source frame
     */
    bool waitForTransform(const std::string& f1, const std::string& f2,
                          const ros::Time& useTime, const float& timeout, bool printError);

    /**
     * Checks whether the two poses are equal (using the accuracy).
     * They can be in different frames as long as the tf transforms
     * are provided. If they are, it is also possible to wait for the transform for a certain
     * maximum time before failing.
     * \param maxWaitTransform if >0, this is the maximum time to wait for the
            transform between different frames to arrive.
     * \param useLatestTime The lastest time of both stamps is assumed, or
     *      otherwise the most recent transform available for both
     * \return -1 if no transform between the frames are possible, -2 if
     *      even the wait for it has failed, -3 if we thought we could get
     *      a transform but then the actual transfom failed. 0 if the poses
     *      are not equal, and 1 if the poses are equal within accuracy
     */
    int equalPoses(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2, float accuracy_pos,
                   float accuracy_ori, bool useLatestTime, float maxWaitTransform, bool printErrors = true);

    /**
     * Returns distance of two poses (possibly after transfomring them first) in parameters posDist and angleDist.
     * Other parameters and return value just as in relativePose().
     */
    int poseDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2, float& posDist, float& angleDist,
                     bool useLatestTime, float maxWaitTransform, bool printErrors);

    /**
     * returns the distance from 'origin' to 'other' as a pose (can be seen as a vector/quaternion from 'origin' to 'pose')
     * \return -1 if no transform between the frames are possible, -2 if even the wait for it has
     * failed, -3 if we thought we could get a transform but then the actual transfom failed. 0 on success.
     */
    int relativePose(const geometry_msgs::PoseStamped& origin, const geometry_msgs::PoseStamped& other,
                     geometry_msgs::Pose& result, bool useLatestTime, float maxWaitTransform, bool printErrors = true);

    /**
     * transforms p into frame_id. If you want to use the most recent tf transform available,
     * you have to set the poses time stamp to ros::Time(0).
     * \retval 0  success.
     * \retval -1 if no transform between the frames are possible
     * \revtal -2 if even the wait for it has failed
     * \retval -3 if we thought we could get a transform but then the actual transfom failed.
     */
    int transformPose(const geometry_msgs::PoseStamped& p, const std::string& frame_id, geometry_msgs::PoseStamped& result,
                      float maxWaitTransform, bool printErrors = true);

    /**
     * gets transform between two frames
     * \param f1 target frame
     * \param f2 source frame
     */
    int getTransform(const std::string& f1, const std::string& f2, geometry_msgs::Pose& result,
                     const ros::Time& useTime, float maxWaitTransform, bool printErrors = true);

    /**
     * Apply a transform (given in parameter transform) to the pose. Essentially this does pose=pose*transform.
     */
    static void applyTransform(const geometry_msgs::Pose& transform, geometry_msgs::Pose& pose);

    /**
     * transfers the SimpleClientGoalState to a ServerGoalHandle
     */
    template<typename GH, typename Res>
    static void mapToGoalHandle(const actionlib::SimpleClientGoalState& state, GH& goalHandle,
                                const Res& res, const std::string& s);

    /**
     * Uses the SimpleClientGoalState to set the ServerGoalHandle to the
     * corresponding state. E.g. if SimpleClientGoalState is rejected,
     * the ServerGoalHandle gets aborted (not also rejected).
     */
    template<typename GH, typename Res>
    static void effectOnGoalHandle(const actionlib::SimpleClientGoalState& state, GH& goalHandle,
                                   const Res& res, const std::string& s = "");
    /**
     * Like other effectOnGoalHandle(), but without result type and error string.
     */
    template<typename GH>
    static void effectOnGoalHandle(const actionlib::SimpleClientGoalState& state, GH& goalHandle);

private:
    static int hasVal(const std::string& val, const std::vector<std::string>& vec);

    //static lock for general method access
    static baselib_binding::recursive_mutex slock;
    static ROSFunctionsPtr _singleton;

    //tf_listener is thread-safe so doesn't require a mutex.
    tf::TransformListener tf_listener;
};

#include <convenience_ros_functions/ROSFunctions.hpp>

}

#endif // CONVENIENCE_ROS_FUNCTIONS_ROSFUNCTIONS_H
