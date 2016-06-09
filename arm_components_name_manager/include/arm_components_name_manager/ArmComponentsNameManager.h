#ifndef ARM_COMPONENTS_NAME_MANAGER_ARMCOMPONENTSNAMEMANAGER_H
#define ARM_COMPONENTS_NAME_MANAGER_ARMCOMPONENTSNAMEMANAGER_H

#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Manages information about the for a robotic "arm" as defined in the URDF file.

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

#include <string>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

namespace arm_components_name_manager
{

/**
 * Manages information about the for a robotic "arm" as defined in the URDF file.
 * The "robotic arm" is the kinematic chain
 * which is used to reach to objects and grasp or manipulate them.
 * It usually includes a gripper or hand to grasp the objects.
 * This manager can be used to manage all sorts of properties which are specified
 * per-joint or per-link. For example, it can also be used to manage PID values.
 *
 * This class reads its parameters from the ROS parameter server in the function
 * loadParameters().
 * Load the paramters on the ROS parameter server before calling loadParameters(),
 * e.g. as follows from a launch file:
 *
 * ```
 *   <arg name="names_config_file" default="$(find <your-package>)/config/<your-config>.yaml"/>
 *   <rosparam command="load" file="$(arg names_config_file)"/>
 * ```
 *
 * Parameters have to be specified like in this documented .yaml template:
 *
 * ``rosed arm_components_name_manager JointsTemplate.yaml``
 *
 * This class can also be manually initialized with default values which take effect
 * if the ROS parameters are not set. The values may be set manually by setValues() and
 * setControllerNames(); alternatively, a subclass may be derived to provide default values
 * by implementing getDefault* methods. The defaults can then be loaded by loadDefaults().
 *
 *
 * **Optional: Access per-joint PID values**
 *
 * To use this class to manage PID values (which is optional),
 * you have to specify the *effort_controller_names*,
 * *position_controller_names* and *velocity_controller_names* lists in the yaml file.
 * These "controller names" are to be the ones used in another configuration file,
 * which has to be in the format also used by ros_control. For example,
 *  for a velocity controller, this could be a PID values config file (lets name it PIDConfig.yaml):
 *
 * ```
 * <joint_name>_velocity_controller:
 *   type: velocity_controllers/JointVelocityController
 *   joint: <joint_name>
 *   pid: {p: 10, i: 0.0001, d: 0.005}
 * ```
 *
 * In this example, the configuration file (e.g. JointsTemplate.yaml) has to include
 * this:
 *
 * ```
 *    velocity_controller_names:
 *       - <joint_name>_velocity_controller
 * ```
 *
 * The names config file (JointsTemplate.yaml) only needs a list of those joint controller names,
 * not the actual PID values. It is assumed that each robot only has *one* velocity, effort or
 * position controller per joint, which is then referred to globally with the same name.
 * The actual PID values may be loaded on the parameter server *after* an instance
 * of this class has already been created.
 * The names in the lists are only needed to look up the right controller values at the
 * time when functions GetPosGains() and GetVelGains() are called to read from the parameter
 * server.
 * Current limitation: The namespace in the JointsTemplate.yaml config file has to be the same as
 * the namespace used in the PID values PIDConfig.yaml file.
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class ArmComponentsNameManager
{
public:
    /**
     * Initializes joint names by reading from parameter server.
     * \param readParams if true, the method loadParameters() is called from within the constructor.
     *      If false, the paramters may be loaded later by calling function loadParameters().
     * \robot_namespace namespace of the robot which needs to be used in the YAML file
     */
    ArmComponentsNameManager(const std::string& robot_namespace, bool readParams);
    ArmComponentsNameManager(const ArmComponentsNameManager& o);

    /**
     * Reads from the private parameter server according to specification in the example .yaml file.
     * This will override any values which were set by loadDefaults()!
     *
     * \retval -1 none of the parameters was found on ROS parameter server
     * \retval 0 some of the parameters were found on ROS parameter server. The existing values
     *      were loaded.
     * \retval 1 All mandatory parameters found on parameter server and loaded.
     * \retval 2 All mandatory parameters AND controller names found on parameter server and loaded.
     */
    int loadParameters(bool printErrors=true, bool verbose=false);

    /**
     * wait for maximum \e maxWait seconds and keep re-trying to laod parameters by
     * calling loadParameters(). This has been considered successful when returning
     * values greater than \e sufficientSuccessCode.
     * \param sufficientSuccessCode return value of loadParameters() which is considered
     *      a success and can stop the waiting.
     * \param waitStep the amount of seconds to sleep in-between re-checking for parameters
     * \return true if parameters loaded successfully according to \e sufficientSuccessCode,
     *      false if maximum wait time exceeded.
     */
    bool waitToLoadParameters(int sufficientSuccessCode, float maxWait, float waitStep=0.1);

    /**
     * Loads default values as given by the implementation. Controller names have no defaults
     * but can be initialized manually with setControllerNames().
     * \return false if this implementation has no defaults
     */
    bool loadDefaults();

    /**
     * Check if the parameters were loaded.
     * Same return values as loadParameters().
     */
    int parametersLoaded()
    {
        return initParamCode;
    }

    /**
     * returns true if defaults were successfully loaded with loadDefaults()
     */
    bool defaultsLoaded()
    {
        return initWithDefaults;
    }

    /**
     * Joint names as to be used in the default order (e.g. for JointState messsages)
     * \param prepend if not empty, this string is going to be prepended to the joint names
     */
    void getJointNames(std::vector<std::string>& joint_names,
                       bool withGripper, const std::string& prepend = std::string()) const;

    const std::vector<std::string>& getArmJoints() const;

    const std::vector<std::string>& getGripperJoints() const;

    const std::vector<float>& getArmJointsInitPose() const;

    const std::vector<float>& getGripperJointsInitPose() const;

    /**
     * returns all gripper links *excluding* the palm link link
     */
    const std::vector<std::string>& getGripperLinks() const;

    /**
     * Returns the link which counts as the "palm", which is
     * the link to which all gripper joints are attached.
     * This link should be the link directly before the gripper joints.
     * There should be **no other movable joints between the palm link and the gripper joints**.
     * The palm link lies between the last joint in arm_joints and
     * before all joints in gripper_joints.
     */
    const std::string& getPalmLink() const;
    
    /**
     * Returns name of the link which is considered to be the end effector
     * which is used to position the effector before grasping
     * objects. Defaults to palm_link if not specified in ROS parameters.
     */
    const std::string& getEffectorLink() const;

    /**
     * Get all links for the arm, *excluding* the palm link
     */
    const std::vector<std::string>& getArmLinks() const;


    /**
     * Joint names as to be used in the default order (e.g. for JointState messsages)
     * \param init_poses (optional) if specified, you can set the target angles of the
     * joint positions here. The order has to be standard, as returned by getJointNames().
     */
//    void initJointState(sensor_msgs::JointState& js, bool withGripper = true,
//                        const std::vector<float> * init_poses = NULL) const;

    /**
     * Copy names (and optionally data fields) into the JointState \e js.
     * Joint names as to be used in the default order (e.g. for JointState messsages). Existing joint state names
     * will be cleared before being re-initialized.
     * \param mode which names to copy int \js. 0 = all joints (first arm, then gripper joints will be
     *      inserted in \e angles); 1 = only arm joints; 2 = only gripper joints.
     * \param init_vals (optional) if specified, you can set the target values of the
     *  joint positions here. The order has to be according to \e mode.
     * \param copyData specifies which type of data \e init_vals contains: if 0, positions. If 1, velocities. If 2, efforts.
     * \param resetOthers set this to true to reset all positon, velocities and efforts and just resize them to correct size and
     *      initialize them to 0. If \e init_vals is set, this will still override the 0 init values. 
     */
    void copyToJointState(sensor_msgs::JointState& js, int mode, const std::vector<float> * init_vals, int copyData, bool resetAll) const;


    /**
     * Extracts all arm and/or gripper values (according to \e extractData), in the standardized order, from a JointState message.
     * \param mode which data to extract. 0 = all joints (first arm, then gripper joints will be
     *      inserted in \e angles); 1 = only arm joints; 2 = only gripper joints.
     * \param extractData if 0, extract positions. If 1, extract velocities. If 2, extract efforts.
     * \retval false extraction not successfull because not all data was available in joint state \e js
     */
    bool extractFromJointState(const sensor_msgs::JointState& js, int mode, std::vector<float>& data, int extractData) const;
    bool extractFromJointState(const sensor_msgs::JointState& js, int mode, sensor_msgs::JointState& result) const;

    /**
     * Helper function to find out the order of joints given in a names vector.
     * Returns in output vector \e idx the indices of arm and/or gripper joints in the \e joint_names vector.
     *
     * \retval -1 error if not all arm or all gripper joints are present in joint_names.
     * Required are either all arm joints, or all gripper joints, or both arm and gripper joints.
     *
     * \retval 0 if all joints present. Then, \e idx is of size numArmJoints() + numGripperJoints(), and
     *      the indices contain first the arm joints, then the gripper joints.
     * \retval 1 if only arm joints present. Then, \e idx is of size numArmJoints().
     * \retval 2 if only gripper joints present. Then, \e idx is of size numGripperJoints().
     *
     * If only one group is present, the indices for the other group in idx are going to be set to -1.
     */
    int getJointIndices(const std::vector<std::string>& joint_names, std::vector<int>& idx) const;

    /**
     * Like other getJointIndices(), but allows to get joint indices for a particular group, according to \e mode.
     * \param mode if 0, get indices for both arm and gripper joints. If 1, get only arm joints. If 2, get only gripper joints.
     * \return true if all the indicess according to \e mode could be determined.
     */
    bool getJointIndices(const std::vector<std::string>& joint_names, std::vector<int>& idx, int mode) const;

    int numArmJoints() const;
    int numGripperJoints() const;
    int numTotalJoints() const
    {
        return numArmJoints() + numGripperJoints();
    }
    /**
     * Check whether this is a gripper joint
     */
    bool isGripper(const std::string& name) const;

    /**
     * Returns the number of the joint in the arm, or -1 if this is no arm joint.
     * Numbering starts with 0.
     */
    int armJointNumber(const std::string& name) const;

    /**
     * Returns the number of the joint of the grippers, or -1 if this is no gripper joint.
     * Numbering starts with 0.
     */
    int gripperJointNumber(const std::string& name) const;

    /**
     * Reads PID parameters from the ROS parameter server, where they are stored as a dictionary
     * as follows:
     *
     *     pid:
     *         p: <p-value>
     *         i: <i-value>
     *         d: <d-value>
     */
    void ReadPIDValues(const std::string& pidParameterName, float& kp, float& ki, float& kd) const;

    /**
     * Calls ReadPIDValues() for the position controller for the joint specified in jointName.
     * If the joint is not found, the values of
     * paramters kp,ki and kd are left as they are, so ideally they should be
     * initialzied with defaults before calling this function.
     * \return false if the joint 'jointName' is not maintained by this joint manager.
     */
    bool GetPosGains(const std::string& jointName, float& kp, float& ki, float& kd) const;

    /**
     * Like GetPosGains(), but for velocity controller.
     */
    bool GetVelGains(const std::string& jointName, float& kp, float& ki, float& kd) const;


    /**
     * Returns the max values for this joint.
     */
    virtual bool GetMaxVals(const std::string& jointName, float& force, float& velocity) const;


    /**
     * Sets the names fo all joints, links, and initial poses which otherwise would be
     * specified in the YAML file.
     * \param _palm_link name of the palm link. See also description in getPalmLink().
     * \param _arm_joints names of the arm joints *without* the gripper joints.
     * \param _arm_links all links which are in-between (and directly before and after) the arm_joints.
     *      It does however *not* include the palm_link.
     * \param _gripper_joints All joints of the "gripper"
     * \param _gripper_links All links of the "gripper".
     *      It does however *not* include the palm_link.
     * \param _arm_joint_init initial ("Home") pose of the arm joints. Has to be the same
     *  order as \e _arm_joints.
     * \param _gripper_joint_init initial ("Home") pose of the gripper joints. Has to be the same
     *      order as \e _gripper_joints.
     */
    void setValues(const std::string& _palm_link,
                   const std::string& _effector_link,
                   const std::vector<std::string>& _arm_joints, const std::vector<std::string>& _arm_links,
                   const std::vector<std::string>& _gripper_joints, const std::vector<std::string>& _gripper_links,
                   const std::vector<float>& _arm_joint_init, const std::vector<float>& _gripper_joint_init);

    /**
     * Sets the names of the controllers as they would otherwise be specified in the YAML file.
     * \param bool forArm set the names for the arm (if false, for the gripper)
     * \param type type of controller: 0 = effort, 1 = velocity, 2 = position
     */
    void setControllerNames(const std::vector<std::string>& controller_names, bool forArm, int type);

    /**
     * return true in subclasses which provides all the defaults
     * in the getDefault*() methods.
     */
    virtual bool hasDefaults()
    {
        return false;
    }

protected:

    virtual std::string getDefaultPalmLink() const { return std::string(); }
    virtual std::string getDefaultEffectorLink() const { return std::string(); }
    virtual std::vector<std::string> getDefaultArmJoints() const { return std::vector<std::string>(); }
    virtual std::vector<std::string> getDefaultArmLinks() const { return std::vector<std::string>(); }
    virtual std::vector<std::string> getDefaultGripperJoints() const { return std::vector<std::string>(); }
    virtual std::vector<std::string> getDefaultGripperLinks() const { return std::vector<std::string>(); }
    virtual std::vector<float> getDefaultArmJointsInitPose() const { return std::vector<float>(); }
    virtual std::vector<float> getDefaultGripperJointsInitPose() const { return std::vector<float>(); }
    virtual std::vector<float> getDefaultArmJointsMaxVel() const { return std::vector<float>(); }
    virtual std::vector<float> getDefaultArmJointsMaxForce() const { return std::vector<float>(); }
    virtual std::vector<float> getDefaultGripperJointsMaxVel() const { return std::vector<float>(); }
    virtual std::vector<float> getDefaultGripperJointsMaxForce() const { return std::vector<float>(); }

private:

    ArmComponentsNameManager();

    /**
     * Names of the arm joints *without* the gripper joints.
     * These are the joints which are used to move the arm in
     * pre-grasp state, but which are *not* part of the acutal
     * gripper (essentially an arm without the grippers)
     */
    std::vector<std::string> arm_joints;

    /**
     * All links which are in-between (and directly before and after)
     * the arm_joints. It does however *not* include the palm_link
     * because this is specified separately.
     */
    std::vector<std::string> arm_links;

    /**
     * All joints of the "gripper". The gripper is the part of the
     * arm used to grasp/grip objects.
     * Essentially, they are the "gripper joints".
     */
    std::vector<std::string> gripper_joints;


    /**
     * All links which are in-between (and directly before and after)
     * the gripper_joints. It does however *not* include the palm_link
     * because this is specified separately.
     */
    std::vector<std::string> gripper_links;

    /**
     * Name of the palm link. This is the link to which objects
     * that this end effector grasps are symbolically "attached"
     * when the object is grasped.
     * This link should be the link directly before the gripper joints.
     * There should be **no other movable joints between the palm link
     * and the gripper joints**.
     * The palm link lies between the last joint in arm_joints and
     * before all joints in gripper_joints.
     */
    std::string palm_link;
    
    /**
     * name of the link which is considered to be the end effector
     * which is used to position the effector before grasping
     * objects. Defaults to palm_link if not specified.
     */
    std::string effector_link;

    /**
     * initial ("Home") pose of the arm joints. Has to be the same
     * order as arm_joints.
     */
    std::vector<float> arm_joint_init;

    /**
     * initial ("Home") pose of the gripper joints. Has to be the same
     * order as gripper_joints.
     */
    std::vector<float> gripper_joint_init;
   
    /**
     * Maximum velocity of the arm joints
     */ 
    std::vector<float> arm_joint_max_vel;
    /**
     * Maximum force of the arm joints
     */ 
    std::vector<float> arm_joint_max_force;
    /**
     * Maximum velocity of the gripper joints
     */ 
    std::vector<float> gripper_joint_max_vel;
    /**
     * Maximum force of the gripper joints
     */ 
    std::vector<float> gripper_joint_max_force;

    std::vector<std::string> arm_effort_controller_names;
    std::vector<std::string> arm_velocity_controller_names;
    std::vector<std::string> arm_position_controller_names;

    std::vector<std::string> gripper_effort_controller_names;
    std::vector<std::string> gripper_velocity_controller_names;
    std::vector<std::string> gripper_position_controller_names;

    std::string robot_namespace;

    /**
     * Initialization code.
     * value: -1 none of the parameters was found on ROS parameter server
     * value: 0 some of the parameters were found on ROS parameter server. The existing values
     *      were loaded.
     * value: 1 All mandatory parameters found on parameter server and loaded.
     * value: 2 All mandatory parameters AND controller names found on parameter server and loaded.
     */
    int initParamCode;

    /**
     * true if the defaults were loaded.
     */
    bool initWithDefaults;
};
}  // namespace
#endif  // ARM_COMPONENTS_NAME_MANAGER_ARMCOMPONENTSNAMEMANAGER_H
