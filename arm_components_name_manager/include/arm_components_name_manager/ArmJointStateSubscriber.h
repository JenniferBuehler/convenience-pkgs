#ifndef ARM_COMPONENTS_NAME_MANAGER_ARMJOINTSTATESUBSCRIBER_H
#define ARM_COMPONENTS_NAME_MANAGER_ARMJOINTSTATESUBSCRIBER_H

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



#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <arm_components_name_manager/ArmComponentsNameManager.h>
#include <architecture_binding/Thread.h>

namespace arm_components_name_manager
{

/**
 * Helper class which subscribes to sensor_msgs::JointState topic to maintain up-to-date
 * state of the arm.
 * It does the updates only when active, which can be triggered with setActive(true).
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class ArmJointStateSubscriber
{
public:
    ArmJointStateSubscriber(const ArmComponentsNameManager& _manager,
                            ros::NodeHandle& n, const std::string& joint_states_topic);
    ~ArmJointStateSubscriber();

    friend std::ostream& operator<<(std::ostream& o, const ArmJointStateSubscriber& j)
    {
        o << j.toString();
        return o;
    }

    void setActive(bool flag);

    std::vector<float> armAngles(bool& valid) const;

    std::vector<float> gripperAngles(bool& valid) const;

    std::string toString() const;
private:
    typedef architecture_binding::unique_lock<architecture_binding::mutex>::type unique_lock;

    void callback(const sensor_msgs::JointState& msg);

    mutable architecture_binding::mutex mutex;
    bool valid_arm, valid_grippers;

    const ArmComponentsNameManager jointsManager;

    std::vector<float> arm_angles;
    std::vector<float> gripper_angles;

    ros::NodeHandle node;
    ros::Subscriber js;

    bool isActive;
};
}  // namespace
#endif  // ARM_COMPONENTS_NAME_MANAGER_ARMJOINTSTATESUBSCRIBER_H
