#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Maintains up-to-date state of the arm.

   Copyright (C) 2015 Jennifer Buehler

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/
#endif

#include <arm_components_name_manager/ArmJointStateSubscriber.h>

#define CHECK_UPDATE_RATE 50

using arm_components_name_manager::ArmJointStateSubscriber;

ArmJointStateSubscriber::ArmJointStateSubscriber(const ArmComponentsNameManager& _manager,
        ros::NodeHandle& n, const std::string& joint_states_topic):
    jointsManager(_manager),
    node(n),
    valid_arm(false),
    valid_grippers(false),
    // subscriber(n),
    subscriberActive(false),
    last_update_time(0)
{
    subscriber = node.subscribe(joint_states_topic, 1000, &ArmJointStateSubscriber::callback, this);
/*    subscriber.start(joint_states_topic);
    update_connection = n.createTimer(ros::Duration(1.0 / CHECK_UPDATE_RATE), &ArmJointStateSubscriber::process, this);*/
}
ArmJointStateSubscriber::~ArmJointStateSubscriber() {}

void ArmJointStateSubscriber::setActive(bool flag)
{
    unique_lock lock(mutex);
    subscriberActive = flag;
    // subscriber.setActive(flag);
}

bool ArmJointStateSubscriber::isActive() const
{
    unique_lock lock(mutex);
    return subscriberActive;
    // return subscriber.isActive();
}

bool ArmJointStateSubscriber::waitForUpdate(float timeout, float checkStepTime) const
{
    if (!isActive())
    {
        ROS_ERROR("Called ArmJointStateSubscriber::waitForUpdate() without calling ArmJointStateSubscriber::setActive(true) first");
        return false;
    }
    ros::Time start_time = ros::Time::now();
    float time_waited=0;
    while ((getLastUpdateTime() < start_time) && ((timeout < 0) || (time_waited < timeout)))
    {
        ROS_INFO("ArmJointStateSubscriber: Waiting...");
        // spin once so that callback() may still be called by the subscriber
        // in case the node is not running in multi-threaded mode.
        ros::spinOnce();
        ros::Duration(checkStepTime).sleep();
        ros::Time curr_time = ros::Time::now();
        time_waited = (curr_time - start_time).toSec();
    }
    if (getLastUpdateTime() >= start_time) return true;
    return false;
}

ros::Time ArmJointStateSubscriber::getLastUpdateTime() const
{
    unique_lock lock(mutex);
    return last_update_time;
}

std::vector<float> ArmJointStateSubscriber::armAngles(bool& valid) const
{
    unique_lock lock(mutex);
    valid = valid_arm;
    if (!valid) ROS_WARN("Arm angles were not complete in the last joint state callback");
    return arm_angles;
}

std::vector<float> ArmJointStateSubscriber::gripperAngles(bool& valid) const
{
    unique_lock lock(mutex);
    valid = valid_grippers;
    if (!valid) ROS_WARN("Gripper angles were not complete in the last joint state callback");
    return gripper_angles;
}

std::string ArmJointStateSubscriber::toString() const
{
    unique_lock lock(mutex);
    std::stringstream str;
    str << "Arm (valid=" << valid_arm << "): ";
    for (int i = 0; i < arm_angles.size(); ++i) str << arm_angles[i] << " / ";
    str << " Gripper (valid=" << valid_grippers << "): ";
    for (int i = 0; i < gripper_angles.size(); ++i) str << gripper_angles[i] << " / ";
    return str.str();
}


void ArmJointStateSubscriber::callback(const sensor_msgs::JointState& msg)
//void ArmJointStateSubscriber::process(const ros::TimerEvent& t)
{
    if (!isActive()) return;

    /*
    // get the newest message
    sensor_msgs::JointState msg;
    float rate = 1.0 / CHECK_UPDATE_RATE;
    float timeout = 5 * rate;
    float checkStepTime = timeout / 4.0;
    bool gotMessage = subscriber.waitForNextMessage(msg, timeout, checkStepTime);
    // do another thread loop, no new message has arrived so far
    if (!gotMessage) return;
    // ROS_INFO("Processing...");   
    */

    std::vector<int> idx;
    // Get joint indices in msg.
    // Returns 0 if all joints present, 1 if only arm joints present, and 2 if only gripper joints present.
    int idxGroup = jointsManager.getJointIndices(msg.name, idx);

    if (idxGroup < 0)
    {
        ROS_WARN("Could not obtain indices of the arm joints in joint state, skipping it.");
        return;
    }

    int numArmJnts = jointsManager.numArmJoints();
    int numGripperJnts = jointsManager.numGripperJoints();

    int numJnts = numArmJnts + numGripperJnts;
    if (idx.size() != numJnts)
    {
        ROS_ERROR("Inconsistency: joint_indices should be same size as all arm joints");
        return;
    }   

    unique_lock lock(mutex);
    bool _valid_grippers = false;
    bool _valid_arm = false;

    if ((idxGroup == 0) || (idxGroup == 1))
    {
        arm_angles.assign(numArmJnts, 0);
        bool allFound = true;
        for (int i = 0; i < numArmJnts; ++i)
        {
            if (idx[i] < 0)
            {
                ROS_ERROR_STREAM("Arm joint "<<i<<" was not in joint state");
                allFound = false;
                continue;
            }
            if (msg.position.size() <= idx[i])
            {
                ROS_ERROR_STREAM("Inconsistency: position size in message is "
                    <<msg.position.size()<<"< tying to index "<<idx[i]);
                allFound = false;
                continue;
            }
            arm_angles[i] = msg.position[idx[i]];
        }
        _valid_arm = allFound;
    }

    if ((idxGroup == 0) || (idxGroup == 2))
    {
        gripper_angles.assign(numGripperJnts, 0);
        bool allFound = true;
        int startIt = 0;
        if (idxGroup == 0) startIt = numArmJnts;
        for (int i = startIt; i < numJnts; ++i)
        {
            if (idx[i] < 0)
            {
                ROS_ERROR_STREAM("Gripper joint " << (i - startIt) << " was not in joint state");
                allFound = false;
                continue;
            }
            if ((i-startIt) >= gripper_angles.size())
            {
                ROS_ERROR_STREAM("Consistency: index of gripper is too large. startIt="<<startIt<<", i="<<i);
                allFound = false;
                continue;
            }
            if (msg.position.size() <= idx[i])
            {
                ROS_ERROR_STREAM("Inconsistency: position size in message is "
                    <<msg.position.size()<<"< tying to index "<<idx[i]);
                allFound = false;
                continue;
            }
            // ROS_INFO_STREAM("Grippers at "<<i-startIt);
            // ROS_INFO_STREAM("msg pos: "<<msg.position[idx[i]]);
            gripper_angles[i-startIt] = msg.position[idx[i]];
        }
        _valid_grippers = allFound;
    }

    if (_valid_grippers || _valid_arm)
    {
        // Some joints were present, so assuming that we got a message related to this arm.
        // Otherwise, maybe this was just a joint state from another package.
        // Then, leave the current states untouched.
        valid_grippers = _valid_grippers;
        valid_arm = _valid_arm;
    }
    last_update_time = ros::Time::now();
}
