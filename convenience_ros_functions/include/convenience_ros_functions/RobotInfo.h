#ifndef CONVENIENCE_ROS_FUNCTIONS_ROBOTINFO
#define CONVENIENCE_ROS_FUNCTIONS_ROBOTINFO

#include <ros/ros.h>
#include <baselib_binding/Thread.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/MultiDOFJointState.h>
#include <convenience_ros_functions/ROSFunctions.h>

namespace convenience_ros_functions {

/**
 * A helper class which includes convenience function to access the most recent information
 * of the robot state. This class is not continuously subscribed to topics. The subscribers
 * are launched on-demand, so it is suitable if you only need the states now and then, and
 * not continuously.
 *
 * \author Jennifer Buehler
 */
class RobotInfo {

    public:
    /**
     * \param default_robot_pose_topic the default robot pose topic. This allows convenience
     *      functions to be called without specifying the topic every time.
     * \param default_joint_states_topic the default joint states topic. This allows convenience
     *      functions to be called without specifying the topic every time.
     */
    RobotInfo(const std::string& default_robot_pose_topic="/amcl_pose",
        const std::string& default_joint_states_topic="/joint_states");

    ~RobotInfo();

    geometry_msgs::PoseStamped getCurrentRobotPose(ros::NodeHandle& n);
    geometry_msgs::PoseStamped getCurrentRobotPose(const std::string& topicName, ros::NodeHandle& n); 
    
    geometry_msgs::PoseWithCovarianceStamped getCurrentRobotPoseWithCovariance(ros::NodeHandle& n); 
    geometry_msgs::PoseWithCovarianceStamped getCurrentRobotPoseWithCovariance(const std::string& topicName, ros::NodeHandle& n); 

    sensor_msgs::JointState getCurrentJointState(ros::NodeHandle& n);
    sensor_msgs::JointState getCurrentJointState(const std::string& topicName, ros::NodeHandle& n);

    /**
     * Transforms the \e robotPose into \e frameID and then constructs a sensor_msgs::MultiDOFJointState
     * using this transformed pose.
     *
     * Background info: MoveIt likes to have the MultiDOFJointState in the frame specified at configuration
     * time (e.g. odom) and seems to have trouble converting it. So do it here.
     * If this frame changes, pass it into "frame_id" parameter.
     */
    static sensor_msgs::MultiDOFJointState getVirtualJointState(const geometry_msgs::PoseStamped& robotPose,
        const std::string& virtualJointName, const std::string& frameID); 

    /**
     * get the current robot pose, and get it as a MulitDOFJointState, transformed to the most recent transform available in frame_id
     */
    sensor_msgs::MultiDOFJointState getCurrentVirtualJointState(const std::string& poseTopicName,
        ros::NodeHandle& n, const std::string& virtualJointName, const std::string& frameID); 

    private:
    typedef baselib_binding::mutex mutex;
    typedef baselib_binding::unique_lock<mutex>::type unique_lock;

    bool receivedJointState();
    void jointStateCallback(const sensor_msgs::JointState& jointstate); 
    bool receivedRobotPose();
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg); 

    mutex poseMutex;
    bool received_rp;
    geometry_msgs::PoseWithCovarianceStamped pose;

    mutex jointStateMutex;
    sensor_msgs::JointState jointState;
    bool received_js;

    std::string robot_pose_topic;
    std::string joint_states_topic;
};

}
#endif  // CONVENIENCE_ROS_FUNCTIONS_ROBOTINFO
