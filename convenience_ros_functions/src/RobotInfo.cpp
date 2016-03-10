#include <convenience_ros_functions/RobotInfo.h>
#include <convenience_ros_functions/ROSFunctions.h>

using convenience_ros_functions::RobotInfo;
using convenience_ros_functions::ROSFunctions;

RobotInfo::RobotInfo(const std::string& _robot_pose_topic,
    const std::string& _joint_states_topic):
    robot_pose_topic(_robot_pose_topic),
    joint_states_topic(_joint_states_topic)
{}

RobotInfo::~RobotInfo(){}

geometry_msgs::PoseStamped RobotInfo::getCurrentRobotPose(ros::NodeHandle& n) {
    return getCurrentRobotPose(robot_pose_topic,n);
}

geometry_msgs::PoseStamped RobotInfo::getCurrentRobotPose(const std::string& topicName, ros::NodeHandle& n) {
    geometry_msgs::PoseWithCovarianceStamped p=getCurrentRobotPoseWithCovariance(topicName,n);
    geometry_msgs::PoseStamped ret;
    ret.header=p.header;
    ret.pose=p.pose.pose;
    return ret;
}

geometry_msgs::PoseWithCovarianceStamped RobotInfo::getCurrentRobotPoseWithCovariance(const std::string& topicName, ros::NodeHandle& n) {
    ros::Subscriber jsub = n.subscribe(topicName, 1000, &RobotInfo::poseCallback,this);
    //ROS_DEBUG("Waiting for robot pos to come in.");
    // Spin on subscription
    poseMutex.lock();
    received_rp=false; 
    poseMutex.unlock();

    ros::Duration spinWait(0.05);
    while (!receivedRobotPose()) {
        ros::spinOnce();
        spinWait.sleep();
    }

    //ROS_DEBUG("Received pose.");
    poseMutex.lock();
    geometry_msgs::PoseWithCovarianceStamped p=pose;
    poseMutex.unlock();
    return p;
}

geometry_msgs::PoseWithCovarianceStamped RobotInfo::getCurrentRobotPoseWithCovariance(ros::NodeHandle& n) {
    return getCurrentRobotPoseWithCovariance(robot_pose_topic,n);
}
        
sensor_msgs::JointState RobotInfo::getCurrentJointState(const std::string& topicName, ros::NodeHandle& n){
    jointStateMutex.lock();
    received_js=false;
    jointStateMutex.unlock();

    ros::Subscriber jsub = n.subscribe(topicName, 10, &RobotInfo::jointStateCallback,this);
    //ROS_INFO("Waiting until current joint state arrives...");

    ros::Duration spinWait(0.05);
    while (!receivedJointState()) {
        ros::spinOnce();
        spinWait.sleep();
    }
    
    //ROS_INFO("Joint state received.");

    jointStateMutex.lock();
    sensor_msgs::JointState js=jointState;
    jointStateMutex.unlock();
    return js;
}

sensor_msgs::JointState RobotInfo::getCurrentJointState(ros::NodeHandle& n){
    return getCurrentJointState(joint_states_topic,n);
}



sensor_msgs::MultiDOFJointState RobotInfo::getVirtualJointState(const geometry_msgs::PoseStamped& robotPose,
    const std::string& virtualJointName, const std::string& frame_id) {

    //ROS_INFO_STREAM("Getting virtual joint state for robot pose "<<robotPose);

    geometry_msgs::PoseStamped _robotPose=robotPose;    
    _robotPose.header.stamp=ros::Time(0); //get the earliest possible transform, so set the time to 0
    if (_robotPose.header.frame_id!=frame_id) {
        int ret=ROSFunctions::Singleton()->transformPose(_robotPose,frame_id,_robotPose,2,true);
        if (ret!=0){
            ROS_WARN("getVirtualJointState(): Failed to transform robot pose into %s.",frame_id.c_str());
        }
    }


    sensor_msgs::MultiDOFJointState j;
    j.header=_robotPose.header;
    //j.header.stamp=ros::Time::now();
    //j.header.frame_id=_robotPose.header.frame_id;
    j.joint_names.push_back(virtualJointName);
    geometry_msgs::Transform t;
    t.translation.x=_robotPose.pose.position.x;
    t.translation.y=_robotPose.pose.position.y;
    t.translation.z=_robotPose.pose.position.z;
    t.rotation=_robotPose.pose.orientation;
    j.transforms.push_back(t);

    //calculate theta from quaternion
    /*Eigen::Quaterniond targetOri;
    tf::quaternionMsgToEigen (_robotPose.pose.pose.orientation, targetOri);
    Eigen::Vector3d targetVector=targetOri*origVector;
    double dist = angularDistanceInXYSigned(origVector,targetVector);
    j.position.push_back(dist);
    */
    return j;

}

sensor_msgs::MultiDOFJointState RobotInfo::getCurrentVirtualJointState(const std::string& poseTopicName, ros::NodeHandle& n, const std::string& virtualJointName, const std::string& frame_id) {
    
    geometry_msgs::PoseStamped robPos=getCurrentRobotPose(poseTopicName, n);
    return getVirtualJointState(robPos,virtualJointName,frame_id);
}

bool RobotInfo::receivedJointState(){
    unique_lock lock(jointStateMutex);
    return received_js;
}

void RobotInfo::jointStateCallback(const sensor_msgs::JointState& jointstate) {
    unique_lock lock(jointStateMutex);
    jointState=jointstate;
    received_js=true;
}

bool RobotInfo::receivedRobotPose(){
    unique_lock lock(poseMutex);
    return received_rp;
}

void RobotInfo::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
    unique_lock lock(poseMutex);
    //ROS_INFO_STREAM("got current pose: "<<msg.pose.pose.position.x<<", "<<msg.pose.pose.position.y<<", "<<msg.pose.pose.position.z);
    pose=msg;
    received_rp=true;
}

