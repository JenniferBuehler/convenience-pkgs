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

using arm_components_name_manager::ArmJointStateSubscriber;

ArmJointStateSubscriber::ArmJointStateSubscriber(const ArmComponentsNameManager& _manager,
        ros::NodeHandle& n, const std::string& joint_states_topic):
    jointsManager(_manager), node(n), valid_arm(false), valid_grippers(false), isActive(false)
{
    js = node.subscribe(joint_states_topic, 1000, &ArmJointStateSubscriber::callback, this);
}
ArmJointStateSubscriber::~ArmJointStateSubscriber() {}

void ArmJointStateSubscriber::setActive(bool flag)
{
    isActive = flag;
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
{
    if (!isActive) return;

    std::vector<int> idx;
    // Get joint indices in msg.
    // Returns 0 if all joints present, 1 if only arm joints present, and 2 if only gripper joints present.
    int idxGroup = jointsManager.getJointIndices(msg.name, idx);

    unique_lock lock(mutex);
    bool _valid_grippers = false;
    bool _valid_arm = false;
    if ((idxGroup == 0) || (idxGroup == 1))
    {
        if (arm_angles.size() < idx.size()) arm_angles.resize(idx.size(), 0);
        for (int i = 0; i < idx.size(); ++i)
        {
            arm_angles[i] = msg.position[idx[i]];
        }
        _valid_arm = true;
    }

    if ((idxGroup == 0) || (idxGroup == 2))
    {
        if (gripper_angles.size() < idx.size()) gripper_angles.resize(idx.size(), 0);
        for (int i = 0; i < idx.size(); ++i)
        {
            gripper_angles[i] = msg.position[idx[i]];
        }
        _valid_grippers = true;
    }
    if (_valid_grippers || _valid_arm)
    {
        // Some joints were present, so assuming that we got a message related to this arm.
        // Otherwise, maybe this was just a joint state from another package.
        // Then, leave the current states untouched.
        valid_grippers = _valid_grippers;
        valid_arm = _valid_arm;
    }
}
