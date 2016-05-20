#include <convenience_ros_functions/ROSFunctions.h>
#include <convenience_math_functions/MathFunctions.h>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#define INIT_SINGLETON_SLEEP_SECS 0.5

using convenience_ros_functions::ROSFunctions;

ROSFunctions::ROSFunctionsPtr ROSFunctions::_singleton;
baselib_binding::recursive_mutex ROSFunctions::slock;

ROSFunctions::ROSFunctions(float tf_max_cache_time):
    tf_listener(ros::Duration(tf_max_cache_time))
{
}

void ROSFunctions::initSingleton()
{
    slock.lock();
    if (!_singleton.get())
    {
        _singleton = ROSFunctionsPtr(new ROSFunctions());
        if (!ros::ok())
        {
            ROS_ERROR("Calling ROSFunctions::initSingleton() outside a valid ROS context. This will likely lead to problems.");
            return;
        }
        ros::Duration(INIT_SINGLETON_SLEEP_SECS).sleep();
    }
    slock.unlock();
}
void ROSFunctions::destroySingleton()
{
    slock.lock();
    if (_singleton.get()) _singleton = ROSFunctionsPtr((ROSFunctions*)NULL);
    slock.unlock();
}

ROSFunctions::ROSFunctionsPtr ROSFunctions::Singleton()
{
    slock.lock();
    if (!_singleton.get())
    {
        ROS_WARN_STREAM("Calling ROSFunctions::Singleton() without having called ROSFunctions::initSingleton()"
            <<" before. This will incur a small wait by ROSFunctions::initSingleton() which is required"
            <<" to wait a bit for the tf listener to get running and avoid problems with tf transforms.");
        initSingleton(); //_singleton = ROSFunctionsPtr(new ROSFunctions()); //first access to helpers, so create it
    }
    slock.unlock();
    return ROSFunctionsPtr(_singleton);
}


void ROSFunctions::applyTransform(const geometry_msgs::Pose& transform, geometry_msgs::Pose& pose)
{
    pose.position.x += transform.position.x;
    pose.position.y += transform.position.y;
    pose.position.z += transform.position.z;

    Eigen::Quaterniond qPose, qTrans, qRes;
    tf::quaternionMsgToEigen(pose.orientation, qPose);
    tf::quaternionMsgToEigen(transform.orientation, qTrans);

    qRes = qPose * qTrans;

    tf::quaternionEigenToMsg(qTrans, pose.orientation);
}

bool ROSFunctions::canGetTransform(const std::string& f1, const std::string& f2,
                                   const ros::Time& useTime, bool printError) const
{

    if (f1.empty() || f2.empty())
    {
        if (printError) ROS_ERROR("Frame ID's must be both set");
        return false;
    }

    if (!tf_listener.frameExists(f1) || !tf_listener.frameExists(f2))
    {
        //if (printError) ROS_ERROR("Can't transform, as a frame does not exist (yet): %i, %i",tf_listener.frameExists(f1),tf_listener.frameExists(f2));
        return false;
    }

    std::string error_msg;
    if (!tf_listener.canTransform(f1, f2, useTime, &error_msg))
    {
        //if (printError) ROS_ERROR("Can't transform between frames %s and %s: %s",f1.c_str(),f2.c_str(),error_msg.c_str());
        return false;
    }

    return true;
}

bool ROSFunctions::canGetTransform(const std_msgs::Header& p1, const std_msgs::Header& p2,
                                   bool latest, bool printError) const
{
    float _time;
    if (latest) _time = std::max(p1.stamp.toSec(), p2.stamp.toSec());
    else        _time = 0; //std::min(p1.stamp.toSec(),p2.stamp.toSec());
    ros::Time useTime(_time);
    return canGetTransform(p1.frame_id, p2.frame_id, useTime, printError);
}


bool ROSFunctions::waitForTransform(const std::string& f1, const std::string& f2,
                                    const ros::Time& useTime, const float& timeout, bool printError)
{
    if (f1.empty() || f2.empty())
    {
        if (printError) ROS_ERROR("Frame ID's must be both set");
        return false;
    }

    ros::Time start = ros::Time::now();
    float waited = 0;
    std::string error_msg;
    //ROS_INFO_STREAM("Waiting for transform using time "<<useTime);
    if (!tf_listener.waitForTransform(f1, f2, useTime, ros::Duration(timeout), ros::Duration(0.01), &error_msg))
    {
        if (printError) ROS_ERROR("Failed to wait for transform between frames %s and %s: tf error msg=%s", f1.c_str(), f2.c_str(), error_msg.c_str());
        return false;
    }
    //if the frames don't exist yet, the wait will just return true, in which case we have to double-wait here.
    while (!tf_listener.frameExists(f1) || !tf_listener.frameExists(f2))
    {
        ros::Duration(0.05).sleep();
        ros::Time t = ros::Time::now();
        waited += (t - start).toSec();
        if (timeout > 1e-04)
        {
            if (waited > timeout) break;
        }
        if (!tf_listener.waitForTransform(f1, f2, useTime,
                                          ros::Duration(timeout > 1e-04 ? timeout - waited : timeout), ros::Duration(0.01), &error_msg))
        {
            if (printError)
                ROS_ERROR("Failed to wait for transform between NEW frames %s and %s: %s",
                          f1.c_str(), f2.c_str(), error_msg.c_str());
            return false;
        }
    }

    return true;
}



bool ROSFunctions::waitForTransform(const std_msgs::Header& p1, const std_msgs::Header& p2,
                                    const float& timeout, bool latest, bool printError)
{
    float _time;
    if (latest) _time = std::max(p1.stamp.toSec(), p2.stamp.toSec());
    else        _time = 0; //std::min(p1.stamp.toSec(),p2.stamp.toSec());
    ros::Time useTime(_time);

    return waitForTransform(p1.frame_id, p2.frame_id, useTime, timeout, printError);
}




int ROSFunctions::equalPoses(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2, float accuracy_pose,
                             float accuracy_rad, bool useLatestTime, float maxWaitTransform, bool printErrors)
{
    geometry_msgs::Pose rel;
    int ret = relativePose(p1, p2, rel, useLatestTime, maxWaitTransform, printErrors);
    if (ret < 0) return ret;

    //ROS_INFO_STREAM("Relative pose: "<<rel);

    Eigen::Vector3d dist;
    Eigen::Quaterniond ori;
    tf::pointMsgToEigen(rel.position, dist);
    tf::quaternionMsgToEigen(rel.orientation, ori);

    double angleDist = convenience_math_functions::MathFunctions::capToPI(Eigen::AngleAxisd(ori).angle());
    //double angleDist = convenience_math_functions::MathFunctions::quatAngularDistance(p1.pose.orientation, p2.pose.orientation);

    //ROS_INFO("Rotation is %f %f %f %f",transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());
    //ROS_INFO("ROSFunctions::equalPoses Distances: %f / %f",dist.norm(),angleDist);

    return (dist.norm() <= accuracy_pose) && (fabs(angleDist) <= accuracy_rad) ? 1 : 0;
}

int ROSFunctions::poseDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2, float& posDist, float& angleDist,
                               bool useLatestTime, float maxWaitTransform, bool printErrors)
{


    geometry_msgs::Pose rel;
    int ret = relativePose(p1, p2, rel, useLatestTime, maxWaitTransform, printErrors);
    if (ret < 0) return ret;

    //ROS_INFO_STREAM("Relative pose: "<<rel);

    Eigen::Vector3d dist;
    Eigen::Quaterniond ori;
    tf::pointMsgToEigen(rel.position, dist);
    tf::quaternionMsgToEigen(rel.orientation, ori);

    angleDist = convenience_math_functions::MathFunctions::capToPI(Eigen::AngleAxisd(ori).angle());
    posDist = dist.norm();
    //ROS_INFO("Rotation is %f %f %f %f",transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());
    //ROS_INFO("ROSFunctions::equalPoses Distances: %f / %f",dist.norm(),angleDist);

    return 0;
}

int ROSFunctions::relativePose(const geometry_msgs::PoseStamped& origin, const geometry_msgs::PoseStamped& other,
                               geometry_msgs::Pose& result, bool useLatestTime, float maxWaitTransform, bool printErrors)
{

    if (origin.header.frame_id.empty() || other.header.frame_id.empty())
    {
        if (printErrors) ROS_ERROR("Frame ID's must be both set");
        return -1;
    }

    float _time;
    if (useLatestTime)
        _time = std::max(origin.header.stamp.toSec(), other.header.stamp.toSec());
    else    _time = 0; //std::min(origin.header.stamp.toSec(),other.header.stamp.toSec());
    ros::Time useTime(_time);


    if (!canGetTransform(origin.header.frame_id, other.header.frame_id, useTime, printErrors))
    {
        if (maxWaitTransform > 0)
        {
            //ROS_INFO("ROSFunctions::relativePose: Waiting for tf transform");
            if (!waitForTransform(origin.header.frame_id, other.header.frame_id, useTime, maxWaitTransform, printErrors))
            {
                if (printErrors) ROS_ERROR("Could not wait for the transform");
                return -2;
            }
            //ROS_INFO("Transform arrived.");
        }
        else
        {
            if (printErrors) ROS_ERROR("Could not get the transform to get the relative pose");
            return -1;
        }
    }

    geometry_msgs::PoseStamped transTo;
    geometry_msgs::PoseStamped _other = other;
    _other.header.stamp = useTime;
    try
    {
        tf_listener.transformPose(origin.header.frame_id, _other, transTo);
    }
    catch (tf::TransformException ex)    //tf::LookupException, tf::ConnectivityException, tf::MaxDepthException, tf::ExtrapolationException  tf::InvalidArgument
    {
        if (printErrors) ROS_ERROR("Could not get transform: %s", ex.what());
        return -3;
    }

    result.position.x = transTo.pose.position.x - origin.pose.position.x;
    result.position.y = transTo.pose.position.y - origin.pose.position.y;
    result.position.z = transTo.pose.position.z - origin.pose.position.z;

    Eigen::Quaterniond ori1, ori2;
    tf::quaternionMsgToEigen(origin.pose.orientation, ori1);
    tf::quaternionMsgToEigen(transTo.pose.orientation, ori2);
    Eigen::Quaterniond relOri = convenience_math_functions::MathFunctions::getRotationFromTo(ori1, ori2);
    tf::quaternionEigenToMsg(relOri, result.orientation);

    return 0;

}

int ROSFunctions::transformPose(const geometry_msgs::PoseStamped& p, const std::string& frame_id,
                                geometry_msgs::PoseStamped& result, float maxWaitTransform, bool printErrors)
{
    if (p.header.frame_id.empty() || frame_id.empty())
    {
        if (printErrors) ROS_ERROR("Frame ID's must be both set");
        return -1;
    }
    if (!canGetTransform(p.header.frame_id, frame_id, ros::Time(0), printErrors))
    {
        if (maxWaitTransform > 0)
        {
            //ROS_INFO("ROSFunctions::relativePose: Waiting for tf transform");
            if (!waitForTransform(p.header.frame_id, frame_id, ros::Time(0), maxWaitTransform, printErrors))
            {
                if (printErrors) ROS_ERROR("transformPose(): Could not wait for the transform");
                return -2;
            }
            //ROS_INFO("Transform arrived.");
        }
        else
        {
            if (printErrors) ROS_ERROR("transformPose(): Could not get the transform.");
            return -1;
        }
    }

    geometry_msgs::PoseStamped transTo;
    try
    {
        tf_listener.transformPose(frame_id, p, transTo);
    }
    catch (tf::TransformException ex)    //tf::LookupException, tf::ConnectivityException, tf::MaxDepthException, tf::ExtrapolationException  tf::InvalidArgument
    {
        if (printErrors) ROS_ERROR("ROSFunctions::transformPose(): Could not get transform: %s", ex.what());
        return -3;
    }
    result = transTo;
    return 0;
}


int ROSFunctions::getTransform(const std::string& f1, const std::string& f2, geometry_msgs::Pose& result,
                               const ros::Time& useTime, float maxWaitTransform, bool printErrors)
{
    if (f1.empty() || f2.empty())
    {
        if (printErrors) ROS_ERROR("Frame ID's must be both set");
        return -1;
    }
    if (!canGetTransform(f1, f2, useTime, printErrors))
    {
        if (maxWaitTransform > 0)
        {
            //ROS_INFO("ROSFunctions::getTransform: Waiting for tf transform");
            if (!waitForTransform(f1, f2, useTime, maxWaitTransform, printErrors))
            {
                if (printErrors) ROS_ERROR("Could not wait for the transform");
                return -2;
            }
            //ROS_INFO("Transform arrived.");
        }
        else
        {
            if (printErrors) ROS_ERROR("Could not get the transform to get the relative pose");
            return -1;
        }
    }

    tf::StampedTransform transform;

    try
    {
        tf_listener.lookupTransform(f1, f2, useTime, transform);
    }
    catch (tf::TransformException ex)    //tf::LookupException, tf::ConnectivityException, tf::MaxDepthException, tf::ExtrapolationException  tf::InvalidArgument
    {
        if (printErrors) ROS_ERROR("Could not get transform: %s", ex.what());
        return -3;
    }

    result.position.x = transform.getOrigin().x();
    result.position.y = transform.getOrigin().y();
    result.position.z = transform.getOrigin().z();
    result.orientation.x = transform.getRotation().x();
    result.orientation.y = transform.getRotation().y();
    result.orientation.z = transform.getRotation().z();
    result.orientation.w = transform.getRotation().w();

    return 0;

}


/*tf::Quaternion getTFRotationFromTo(const tf::Quaternion& q1, const tf::Quaternion& q2) {
    tf::Quaternion ret= q2*q1.inverse();
    ret.normalize();
    return ret;
}*/


void ROSFunctions::assignJointState(const sensor_msgs::JointState& target_joints, sensor_msgs::JointState& joint_state)
{
    for (int i = 0; i < joint_state.name.size(); ++i)
    {
        int idx = hasVal(joint_state.name[i], target_joints.name);
        if (idx < 0) continue;
        joint_state.position[i] = target_joints.position[idx];
        joint_state.velocity[i] = target_joints.velocity[idx];
        joint_state.effort[i] = target_joints.effort[idx];
    }

    for (int i = 0; i < target_joints.name.size(); ++i)
    {
        int idx = hasVal(target_joints.name[i], joint_state.name);
        if (idx >= 0) continue;

        joint_state.name.push_back(target_joints.name[i]);
        joint_state.position.push_back(target_joints.position[i]);
        joint_state.velocity.push_back(target_joints.velocity[i]);
        joint_state.effort.push_back(target_joints.effort[i]);
    }

}


bool ROSFunctions::intersectJointState(const sensor_msgs::JointState& s1, const sensor_msgs::JointState& s2, sensor_msgs::JointState& result,
                                       bool init_s1, bool s2_is_subset)
{
    result = s1;
    for (int i = 0; i < s1.name.size(); ++i)
    {
        int idx = hasVal(s1.name[i], s2.name);
        if (idx < 0)
        {
            if (s2_is_subset)
            {
                ROS_ERROR_STREAM("Joint states do not have name " << s1.name[i]);
                return false;
            }
            continue;
        }
        if (result.name[i] != s2.name[idx])
        {
            ROS_ERROR("ROSFunctions::copyJointStates consistency error!");
            return false;
        }

        if (init_s1)   //no need to continue because result already has values of s1.
        {
            continue;
        }

        result.position[i] = s2.position[idx];
        result.velocity[i] = s2.velocity[idx];
        result.effort[i] = s2.effort[idx];
    }
    return true;
}

bool ROSFunctions::intersectJointStates(const sensor_msgs::JointState& s1, const sensor_msgs::JointState& s2,
                                        sensor_msgs::JointState& result, bool s2_is_subset)
{
    result = s1;
    for (int i = 0; i < s1.name.size(); ++i)
    {
        int idx = hasVal(s1.name[i], s2.name);
        if (idx < 0)
        {
            if (s2_is_subset)
            {
                ROS_ERROR_STREAM("Joint states do not have name " << s1.name[i]);
                return false;
            }
            continue;
        }
        if (result.name[i] != s2.name[idx])
        {
            ROS_ERROR("ROSFunctions::copyJointStates consistency error!");
            return false;
        }
        result.position[i] = s2.position[idx];
        result.velocity[i] = s2.velocity[idx];
        result.effort[i] = s2.effort[idx];
    }
    return true;
}


bool ROSFunctions::equalJointPositionsSimple(const sensor_msgs::JointState& j1,
                                   const sensor_msgs::JointState& j2, const float pos_tolerance)
{
    for (int i = 0; i < j1.name.size(); ++i)
    {
        if (i >= j2.name.size()) break;
        if (j1.name[i] != j2.name[i]) return false;
        if (fabs(j1.position[i] - j2.position[i]) > pos_tolerance) return false;
    }
    return true;
}

int ROSFunctions::equalJointPositions(const sensor_msgs::JointState& j1,
                                   const sensor_msgs::JointState& j2, const float pos_tolerance)
{
    /// Probably can be implemented more efficient, but for no this will do

    sensor_msgs::JointState intersectJS;

    if (!intersectJointState(j1, j2, intersectJS, true, true))
    {
        return -2;
    }

    // ROS_INFO_STREAM("Input1: "<<std::endl<<j1<<" Input2: "<<std::endl<<j2<<" intersection "<<std::endl<<intersectJS);

    // intersectJS is not initialized with values in j1.
    // because j2 was required to be subset of j1 in intersectJointState(),
    // now intersectJS and j2 have to be of same size, and have same order of
    // joints as j1.
    if (intersectJS.position.size() != j2.position.size())
    {
        return -3;
    }
    if (!ROSFunctions::equalJointPositionsSimple(intersectJS, j2, pos_tolerance))
    {
        return -1;
    }
    return 1;
}

bool ROSFunctions::getJointStateAt(int idx, const trajectory_msgs::JointTrajectory& traj,
                                   sensor_msgs::JointState& result)
{
    if (traj.points.size() <= idx) return false;
    trajectory_msgs::JointTrajectoryPoint p;
    p = traj.points[idx];
    result.name = traj.joint_names;
    result.position = p.positions;
    result.velocity = p.velocities;
    result.effort = p.effort;
    //result.acceleration=lastPoint.accelerations;
    if (result.position.size() != result.name.size())
    {
        ROS_ERROR("ROSFunctions: Joint state has to have at least positions!");
        return false;
    }
    if (result.velocity.empty())
    {
        for (int i = 0; i < result.name.size(); ++i) result.velocity.push_back(0);
    }
    if (result.effort.empty())
    {
        for (int i = 0; i < result.name.size(); ++i) result.effort.push_back(0);
    }
    if (result.velocity.size() != result.name.size())
    {
        ROS_ERROR("ROSFunctions: Joint state velocities have to be equal size.");
        return false;
    }
    if (result.effort.size() != result.name.size())
    {
        ROS_ERROR("ROSFunctions: Joint state efforts have to be equal size.");
        return false;
    }
    return true;
}

int ROSFunctions::hasVal(const std::string& val, const std::vector<std::string>& vec)
{
    for (int i = 0; i < vec.size(); ++i) if (vec[i] == val) return i;
    return -1;
}


