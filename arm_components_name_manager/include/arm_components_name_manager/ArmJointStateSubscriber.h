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
#include <baselib_binding/Thread.h>

// #include <convenience_ros_functions/TypedSubscriber.h>

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
    ArmJointStateSubscriber(
        const ArmComponentsNameManager& _manager,
        ros::NodeHandle& n,
        const std::string& joint_states_topic);

    ~ArmJointStateSubscriber();

    friend std::ostream& operator<<(std::ostream& o, const ArmJointStateSubscriber& j)
    {
        o << j.toString();
        return o;
    }

    /**
     * Activates or deactivates the processing of incoming messages.
     * This can be used to save a bit of processing time when updates
     * are currently not required.
     */
    void setActive(bool flag);

    /**
     * Waits for the next JointState message to arrive.
     * Only succeeds if active (isActive() returns true)
     * \param timeout the timeout, or negative if no timeout to be used
     * \param checkStepTime time (in seconds) to sleep in-between checks wheter
     *      a new joint state has arrived.
     * \return true if successfully waited until next message or false
     *      if timeout reached (or isActive() returned false)
     */
    bool waitForUpdate(float timeout=-1, float checkStepTime=0.1) const;

    std::vector<float> armAngles(bool& valid) const;

    std::vector<float> gripperAngles(bool& valid) const;

    std::string toString() const;

    /**
     * \return true if subscriber is active. See also setActive(bool).
     */
    bool isActive() const;
private:
    typedef baselib_binding::unique_lock<baselib_binding::recursive_mutex>::type unique_lock;
    // typedef convenience_ros_functions::TypedSubscriber<sensor_msgs::JointState> JointStateSubscriber;

    void callback(const sensor_msgs::JointState& msg);
//    void process(const ros::TimerEvent& t);

    ros::Time getLastUpdateTime() const;

    // protects all fields which need protecting
    mutable baselib_binding::recursive_mutex mutex;
    bool valid_arm, valid_grippers;

    const ArmComponentsNameManager jointsManager;

    std::vector<float> arm_angles;
    std::vector<float> gripper_angles;

    ros::NodeHandle node;

/*    JointStateSubscriber subscriber;
    ros::Timer update_connection;*/
    ros::Subscriber subscriber;
    bool subscriberActive; 
    ros::Time last_update_time;
};
}  // namespace
#endif  // ARM_COMPONENTS_NAME_MANAGER_ARMJOINTSTATESUBSCRIBER_H
