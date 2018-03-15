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


#include <arm_components_name_manager/ArmComponentsNameManager.h>
#include <map>
#include <string>
#include <vector>

using arm_components_name_manager::ArmComponentsNameManager;

ArmComponentsNameManager::ArmComponentsNameManager(const std::string& _robot_namespace,
        bool readParams):
    robot_namespace(_robot_namespace),
    initParamCode(-1),
    initWithDefaults(false)
{
    // get joint names from parameter server
    if (readParams) loadParameters();
}

ArmComponentsNameManager::ArmComponentsNameManager(const ArmComponentsNameManager& o):
    palm_link(o.palm_link),
    effector_link(o.effector_link),
    arm_joints(o.arm_joints),
    arm_links(o.arm_links),
    gripper_joints(o.gripper_joints),
    gripper_links(o.gripper_links),
    arm_joint_init(o.arm_joint_init),
    gripper_joint_init(o.gripper_joint_init),
    arm_effort_controller_names(o.arm_effort_controller_names),
    arm_velocity_controller_names(o.arm_velocity_controller_names),
    arm_position_controller_names(o.arm_position_controller_names),
    gripper_effort_controller_names(o.gripper_effort_controller_names),
    gripper_velocity_controller_names(o.gripper_velocity_controller_names),
    gripper_position_controller_names(o.gripper_position_controller_names),
    initParamCode(o.initParamCode),
    initWithDefaults(o.initWithDefaults)
{
}

bool ArmComponentsNameManager::loadDefaults()
{
    if (!hasDefaults()) return false;
    palm_link = getDefaultPalmLink();
    effector_link = getDefaultEffectorLink();
    arm_joints = getDefaultArmJoints();
    arm_links = getDefaultArmLinks();
    arm_joint_init = getDefaultArmJointsInitPose();
    gripper_joints = getDefaultGripperJoints();
    gripper_links = getDefaultGripperLinks();
    gripper_joint_init = getDefaultGripperJointsInitPose();
    initWithDefaults = true;
    return true;
}



int ArmComponentsNameManager::loadParameters(bool printErrors, bool verbose)
{
    bool allControllersSpecified = true;
    int noSpec = 0;
    int numSpecs = 0;

    ros::NodeHandle robot_nh(robot_namespace);
    ROS_INFO_STREAM("ArmComponentsNameManager reading parameters from namespace: " << robot_nh.getNamespace());

    robot_nh.getParam("palm_link", palm_link);
    if (palm_link.empty())
    {
        ++noSpec;
        if (printErrors) ROS_ERROR("Parameter palm_link should be specified");
    }
    ++numSpecs;
    if (verbose) ROS_INFO_STREAM("Palm link: "<<palm_link);

    robot_nh.getParam("effector_link", effector_link);
    if (effector_link.empty())
    {
        ROS_INFO("INFO: effector_link not specified, defaulting to same as palm_link");
        effector_link = palm_link;
    }
    if (verbose) ROS_INFO_STREAM("Effector link: "<<effector_link);

    // --- arm parameters

    if (verbose) ROS_INFO_STREAM("Reading arm_joints:");
    robot_nh.getParam("arm_joints", arm_joints);
    if (arm_joints.empty())
    {
        ++noSpec;
        if (printErrors) ROS_ERROR("Parameter arm_joints should be specified as an array");
    }
    ++numSpecs;
    if (verbose) for (int i=0; i < arm_joints.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_joints[i]);}

    if (verbose) ROS_INFO_STREAM("Reading arm_joint_init:");
    robot_nh.getParam("arm_joint_init", arm_joint_init);
    if (arm_joint_init.empty())
    {
        ++noSpec;
        if (printErrors) ROS_ERROR("Parameter arm_joint_init should be specified as an array");
    }
    ++numSpecs;
    if (verbose) for (int i=0; i < arm_joint_init.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_joint_init[i]);}

    if (verbose) ROS_INFO_STREAM("Reading arm_joint_max_vel:");
    robot_nh.getParam("arm_joint_max_vel", arm_joint_max_vel);
    if (arm_joint_max_vel.empty())
    {
        ++noSpec;
        if (printErrors) ROS_ERROR("Parameter arm_joint_max_vel should be specified as an array");
    }
    ++numSpecs;
    if (verbose) for (int i=0; i < arm_joint_max_vel.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_joint_max_vel[i]);}

    if (verbose) ROS_INFO_STREAM("Reading arm_joint_max_force:");
    robot_nh.getParam("arm_joint_max_force", arm_joint_max_force);
    if (arm_joint_max_force.empty())
    {
        ++noSpec;
        if (printErrors) ROS_ERROR("Parameter arm_joint_max_force should be specified as an array");
    }
    ++numSpecs;
    if (verbose) for (int i=0; i < arm_joint_max_force.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_joint_max_force[i]);}

    if (verbose) ROS_INFO_STREAM("Reading arm_links:");
    robot_nh.getParam("arm_links", arm_links);
    if (arm_links.empty())
    {
        ++noSpec;
        if (printErrors) ROS_ERROR("Parameter arm_links should be specified as an array");
    }
    ++numSpecs;
    if (verbose) for (int i=0; i < arm_links.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_links[i]);}

    // controllers

    if (verbose) ROS_INFO_STREAM("Reading arm_position_controller_names:");
    robot_nh.getParam("arm_position_controller_names", arm_position_controller_names);
    if (arm_position_controller_names.empty())
    {
        allControllersSpecified = false;
        ROS_INFO("INFO: Parameter arm_position_controller_names has not been specified");
    }
    if (verbose) for (int i=0; i < arm_position_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_position_controller_names[i]);}

    if (verbose) ROS_INFO_STREAM("Reading arm_velocity_controller_names:");
    robot_nh.getParam("arm_velocity_controller_names", arm_velocity_controller_names);
    if (arm_velocity_controller_names.empty())
    {
        allControllersSpecified = false;
        ROS_INFO("INFO: Parameter arm_velocity_controller_names has not been specified");
    }
    if (verbose) for (int i=0; i < arm_velocity_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_velocity_controller_names[i]);}

    if (verbose) ROS_INFO_STREAM("Reading arm_effort_controller_names:");
    robot_nh.getParam("arm_effort_controller_names", arm_effort_controller_names);
    if (arm_effort_controller_names.empty())
    {
        allControllersSpecified = false;
        ROS_INFO("INFO: Parameter arm_effort_controller_names has not been specified");
    }
    if (verbose) for (int i=0; i < arm_effort_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_effort_controller_names[i]);}

    // --- gripper parameters

    if (verbose) ROS_INFO_STREAM("Reading gripper_joints:");
    robot_nh.getParam("gripper_joints", gripper_joints);
    if (gripper_joints.empty())
    {
        // ++noSpec;
        ROS_WARN("Parameter gripper_joints has not been specified");
    }
    // ++numSpecs;
    if (verbose) for (int i=0; i < gripper_joints.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_joints[i]);}

    if (verbose) ROS_INFO_STREAM("Reading gripper_joint_init:");
    robot_nh.getParam("gripper_joint_init", gripper_joint_init);
    if (gripper_joint_init.empty())
    {
        // ++noSpec;
        ROS_WARN("Parameter gripper_joint_init has not been specified");
    }
    // ++numSpecs;
    if (verbose) for (int i=0; i < gripper_joint_init.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_joint_init[i]);}

    if (verbose) ROS_INFO_STREAM("Reading gripper_joint_max_vel:");
    robot_nh.getParam("gripper_joint_max_vel", gripper_joint_max_vel);
    if (gripper_joint_max_vel.empty())
    {
        // ++noSpec;
        ROS_WARN("Parameter gripper_joint_max_vel has not been specified");
    }
    // ++numSpecs;
    if (verbose) for (int i=0; i < gripper_joint_max_vel.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_joint_max_vel[i]);}

    if (verbose) ROS_INFO_STREAM("Reading gripper_joint_max_force:");
    robot_nh.getParam("gripper_joint_max_force", gripper_joint_max_force);
    if (gripper_joint_max_force.empty())
    {
        // ++noSpec;
        ROS_WARN("Parameter gripper_joint_max_force has not been specified");
    }
    // ++numSpecs;
    if (verbose) for (int i=0; i < gripper_joint_max_force.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_joint_max_force[i]);}

    if (verbose) ROS_INFO_STREAM("Reading gripper_links:");
    robot_nh.getParam("gripper_links", gripper_links);
    if (gripper_links.empty())
    {
        // ++noSpec;
        ROS_WARN("Parameter gripper_links has not been specified");
    }
    // ++numSpecs;
    if (verbose) for (int i=0; i < gripper_links.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_links[i]);}

    // controllers

    if (verbose) ROS_INFO_STREAM("Reading gripper_position_controller_names:");
    robot_nh.getParam("gripper_position_controller_names", gripper_position_controller_names);
    if (gripper_position_controller_names.empty())
    {
        allControllersSpecified = false;
        ROS_INFO("INFO: Parameter gripper_position_controller_names has not been specified");
    }
    if (verbose) for (int i=0; i < gripper_position_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_position_controller_names[i]);}

    if (verbose) ROS_INFO_STREAM("Reading gripper_velocity_controller_names:");
    robot_nh.getParam("gripper_velocity_controller_names", gripper_velocity_controller_names);
    if (gripper_velocity_controller_names.empty())
    {
        allControllersSpecified = false;
        ROS_INFO("INFO: Parameter gripper_velocity_controller_names has not been specified");
    }
    if (verbose) for (int i=0; i < gripper_velocity_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_velocity_controller_names[i]);}

    if (verbose) ROS_INFO_STREAM("Reading gripper_effort_controller_names:");
    robot_nh.getParam("gripper_effort_controller_names", gripper_effort_controller_names);
    if (gripper_effort_controller_names.empty())
    {
        allControllersSpecified = false;
        ROS_INFO("INFO: Parameter gripper_effort_controller_names has not been specified");
    }
    if (verbose) for (int i=0; i < gripper_effort_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_effort_controller_names[i]);}

    initParamCode = 0;
    if (noSpec == numSpecs) initParamCode =  -1;
    if ((noSpec == 0) && allControllersSpecified) initParamCode =  2;
    else if (noSpec == 0) initParamCode =  1;
    return initParamCode;
}


bool ArmComponentsNameManager::waitToLoadParameters(int sufficientSuccessCode, float maxWait, float waitStep)
{
    float timeWaited = 0;
    while (timeWaited < maxWait)
    {
        int loadParamRet = loadParameters(false);
        if (loadParamRet >= sufficientSuccessCode)
            return true;
        ROS_INFO("ArmComponentsNameManager: wait for ROS parameters to be loaded....");
        ros::Duration(waitStep).sleep();
        timeWaited += waitStep;
    }
    return false;
}

void ArmComponentsNameManager::ReadPIDValues(const std::string& pidParameterName, float& kp, float& ki, float& kd) const
{
    static const std::string pid_param_name = "pid";

    std::map<std::string, float> pid;
    ros::NodeHandle pub("");
    if (!pub.getParam(robot_namespace + "/" + pidParameterName + "/" + pid_param_name, pid))
    {
        ROS_WARN_STREAM(pidParameterName << " was not on parameter server. Keeping default values.");
    }
    else
    {
        kp = pid["p"];
        ki = pid["i"];
        kd = pid["d"];
    }
}

bool ArmComponentsNameManager::GetPosGains(const std::string& jointName, float& kp, float& ki, float& kd) const
{
    std::vector<std::string>::const_iterator jnt = std::find(arm_joints.begin(), arm_joints.end(), jointName);
    if (jnt == arm_joints.end())
    {
        jnt = std::find(gripper_joints.begin(), gripper_joints.end(), jointName);
        if (jnt == gripper_joints.end())
        {
            ROS_ERROR_STREAM("ArmComponentsNameManager does not maintain joint name '" << jointName << "'");
            return false;
        }
        int idx = jnt - gripper_joints.begin();
        if (gripper_position_controller_names.size() <= idx)
        {
            ROS_ERROR_STREAM("ArmComponentsNameManager does have the name for position controller '" << jointName << "'");
            return false;
        }
        ReadPIDValues(gripper_position_controller_names[idx], kp, ki, kd);
    }
    else
    {
        int idx = jnt - arm_joints.begin();
        if (arm_position_controller_names.size() <= idx)
        {
            ROS_ERROR_STREAM("ArmComponentsNameManager does have the name for position controller '" << jointName << "'");
            return false;
        }
        ReadPIDValues(arm_position_controller_names[idx], kp, ki, kd);
    }
    return true;
}

bool ArmComponentsNameManager::GetVelGains(const std::string& jointName, float& kp, float& ki, float& kd) const
{
    std::vector<std::string>::const_iterator jnt = std::find(arm_joints.begin(), arm_joints.end(), jointName);
    if (jnt == arm_joints.end())
    {
        jnt = std::find(gripper_joints.begin(), gripper_joints.end(), jointName);
        if (jnt == gripper_joints.end())
        {
            ROS_ERROR_STREAM("ArmComponentsNameManager does not maintain joint name '" << jointName << "'");
            return false;
        }
        int idx = jnt - gripper_joints.begin();
        if (gripper_velocity_controller_names.size() <= idx)
        {
            ROS_ERROR_STREAM("ArmComponentsNameManager does have the name for velocity controller '" << jointName << "'");
            return false;
        }
        ReadPIDValues(gripper_velocity_controller_names[idx], kp, ki, kd);
    }
    else
    {
        int idx = jnt - arm_joints.begin();
        if (arm_velocity_controller_names.size() <= idx)
        {
            ROS_ERROR_STREAM("ArmComponentsNameManager does have the name for velocity controller '" << jointName << "'");
            return false;
        }
        ReadPIDValues(arm_velocity_controller_names[idx], kp, ki, kd);
    }
    return true;
}

bool ArmComponentsNameManager::GetMaxVals(const std::string& jointName, float& force, float& velocity) const
{
    std::vector<std::string>::const_iterator jnt = std::find(arm_joints.begin(), arm_joints.end(), jointName);
    if (jnt == arm_joints.end())
    {
        jnt = std::find(gripper_joints.begin(), gripper_joints.end(), jointName);
        if (jnt == gripper_joints.end())
        {
            ROS_ERROR_STREAM("ArmComponentsNameManager does not maintain joint name '" << jointName << "'");
            return false;
        }
        int idx = jnt - gripper_joints.begin();
        if ((gripper_joint_max_vel.size() <= idx) || (gripper_joint_max_force.size() <= idx))
        {
            ROS_ERROR_STREAM("ArmComponentsNameManager does not have all max values for '" << jointName << "'. Will use 0 instead");
            velocity = 0;
            force = 0;
            return true;
        }
        velocity=gripper_joint_max_vel[idx];
        force=arm_joint_max_force[idx];
    }
    else
    {
        int idx = jnt - arm_joints.begin();
        if ((arm_joint_max_vel.size() <= idx) || (arm_joint_max_force.size() <= idx))
        {
            ROS_WARN_STREAM("ArmComponentsNameManager does not have all max values for '" << jointName << "'. Will use 0 instead");
            velocity = 0;
            force = 0;
            return true;
        }
        velocity=arm_joint_max_vel[idx];
        force=arm_joint_max_force[idx];
    }
    return true;
}



void ArmComponentsNameManager::getJointNames(std::vector<std::string>& joint_names, bool withGripper, const std::string& prepend) const
{
    joint_names.insert(joint_names.begin(), arm_joints.begin(), arm_joints.end());
    if (!withGripper) return;
    joint_names.insert(joint_names.end(), gripper_joints.begin(), gripper_joints.end());
    if (!prepend.empty())
    {
        for (std::vector<std::string>::iterator it = joint_names.begin();
                it != joint_names.end(); ++it)
        {
            *it = prepend + *it;
        }
    }
}

const std::vector<std::string>& ArmComponentsNameManager::getGripperLinks() const
{
    return gripper_links;
}

const std::string& ArmComponentsNameManager::getPalmLink() const
{
    return palm_link;
}

const std::string& ArmComponentsNameManager::getEffectorLink() const
{
    return effector_link;
}

const std::vector<std::string>& ArmComponentsNameManager::getArmLinks() const
{
    return arm_links;
}


const std::vector<std::string>& ArmComponentsNameManager::getArmJoints() const
{
    return arm_joints;
}
const std::vector<std::string>& ArmComponentsNameManager::getGripperJoints() const
{
    return gripper_joints;
}

const std::vector<float>& ArmComponentsNameManager::getArmJointsInitPose() const
{
    return arm_joint_init;
}
const std::vector<float>& ArmComponentsNameManager::getGripperJointsInitPose() const
{
    return gripper_joint_init;
}


/*void ArmComponentsNameManager::initJointState(sensor_msgs::JointState& js, bool withGripper, const std::vector<float> * init_poses) const
{
    getJointNames(js.name, withGripper);
    int num = arm_joints.size() + gripper_joints.size();
    if (!withGripper) num = arm_joints.size();
    js.position.resize(num, 0);
    js.velocity.resize(num, 0);
    js.effort.resize(num, 0);
    if (init_poses)
    {
        for (int i = 0; i < num; ++i)
        {
            js.position[i] = (*init_poses)[i];
        }
    }
}*/


void ArmComponentsNameManager::copyToJointState(sensor_msgs::JointState& js, int mode, const std::vector<float> * init_vals, int copyData, bool resetAll) const
{
    // 0 = all joints; 1 = only arm joints; 2 = only gripper joints.
        
    js.name.clear();
    int size = -1;
    if (mode == 0)
    {
        size = arm_joints.size() + gripper_joints.size();
        getJointNames(js.name, true);
    }
    else if (mode == 1)
    {
        size = arm_joints.size();
        js.name = arm_joints;
    }
    else if (mode == 2)
    {
        size = gripper_joints.size();
        js.name = gripper_joints;
    }
       
    if (resetAll)
    {
        js.position.resize(size, 0);
        js.velocity.resize(size, 0);
        js.effort.resize(size, 0);
    }
    if (init_vals)
    {
        if (copyData == 0) js.position.resize(size, 0);
        else if (copyData == 1) js.velocity.resize(size, 0);
        else if (copyData == 2) js.effort.resize(size, 0);
        for (int i = 0; i < size; ++i)
        {
            if (copyData == 0) js.position[i] = (*init_vals)[i];
            else if (copyData == 1) js.velocity[i] = (*init_vals)[i];
            else if (copyData == 2) js.effort[i] = (*init_vals)[i];
        }
    }
}

bool ArmComponentsNameManager::extractFromJointState(const sensor_msgs::JointState& js,
    int mode, std::vector<float>& data, int extractData) const
{
    if ((mode < 0) || mode > 2)
    {
        ROS_ERROR("Consistency: mode has to be in the range 0..2");
        return false;
    }

    std::vector<int> joint_indices;
    int idx = getJointIndices(js.name, joint_indices);
    if (idx < 0)
    {
        ROS_ERROR("Could not obtain indices of the arm joints in joint state");
        return false;
    }

    if ((idx != 0) && (idx != mode))
    {   // joint states does not contain all joints, and
        // it does not contain either arm or gripper
        ROS_ERROR_STREAM("Joint state does not contain the information requested with mode "
            << mode <<" (return code was " << idx << ")");
        return false;
    }

    if (joint_indices.size() != (arm_joints.size() + gripper_joints.size()))
    {
        ROS_ERROR("Inconsistency: joint_indices should be same size as all arm joints");
        return false;
    }   
 
    int startIt = 0;
    int endIt = arm_joints.size() + gripper_joints.size();
    // all joints in joint state, but only sub-state desired
    if (mode == 1)  // get arm
    {
        startIt = 0;
        endIt = arm_joints.size();
    } else if (mode == 2) {  // get gripper
        startIt = arm_joints.size(); 
        endIt = startIt + gripper_joints.size();
    }

    data.clear();
    for (int i = startIt; i < endIt; ++i)
    {
        if (joint_indices[i] < 0)
        {
            ROS_ERROR_STREAM("Joint " << i << " was not in joint state");
            return false;
        }
        if (i >= joint_indices.size())
        {
            ROS_ERROR_STREAM("Inconsistency: joint indices are not of expected size: is "
                <<joint_indices.size()<<", trying to index "<<i);
            return false;
        }
        if (js.position.size() <= joint_indices[i])
        {
            ROS_ERROR_STREAM("Inconsistency: JointState position vector was not of same size as names: "
                <<js.position.size()<<" idx="<<joint_indices[i] << " i = "<<i);
            return false;
        }
        if (extractData == 0) data.push_back(js.position[joint_indices[i]]);
        else if (extractData == 1) data.push_back(js.velocity[joint_indices[i]]);
        else if (extractData == 2) data.push_back(js.effort[joint_indices[i]]);
    }
    return true;
}
    
bool ArmComponentsNameManager::extractFromJointState(const sensor_msgs::JointState& js, int mode, sensor_msgs::JointState& result) const
{
    std::vector<float> pos, vel, eff;
    if (!extractFromJointState(js,mode,pos,0)) return false;
    if (!extractFromJointState(js,mode,vel,1)) return false;
    if (!extractFromJointState(js,mode,eff,2)) return false;
    copyToJointState(result, mode, &pos, 0, false);
    copyToJointState(result, mode, &vel, 1, false);
    copyToJointState(result, mode, &eff, 2, false);
    return true; 
}

bool ArmComponentsNameManager::getJointIndices(const std::vector<std::string>& joint_names, std::vector<int>& idx, int mode) const
{
    if (mode == 0) return getJointIndices(joint_names, idx) >= 0;
    
    // mode has to be 1 or 2
    if ((mode !=1) && (mode != 2))
    {
        ROS_ERROR("Consistency: getJointIndices() must be called with mode 0, 1 or 2");
        return false;
    }

    // get only subset of the indices. Get all indices first,
    // and then shrink the returning indices accordingly.

    std::vector<int> allIdx;
    int allRet = getJointIndices(joint_names, allIdx);
    if (allRet < 0) return false;
    if (allRet == mode)
    {
        idx = allIdx;
        return true;
    }

    idx.clear();
    int numArmJoints = getArmJoints().size();
    if (mode == 1)
    {   // only arm
        idx.insert(idx.begin(),allIdx.begin(), allIdx.begin()+numArmJoints);
    }
    else
    {   // only fingers
        idx.insert(idx.begin(),allIdx.begin()+numArmJoints, allIdx.end());
    }
    return true;
    
}

int ArmComponentsNameManager::getJointIndices(const std::vector<std::string>& joint_names, std::vector<int>& idx) const
{
    typedef std::vector<std::string>::const_iterator It;
    idx.clear();

    bool armIncomplete = false;
    std::vector<int> arm_idx;
    for (int i = 0; i < arm_joints.size(); ++i)
    {
        It jnt = std::find(joint_names.begin(), joint_names.end(), arm_joints[i]);
        if (jnt == joint_names.end())
        {
            armIncomplete = true;
            break;
        }
        arm_idx.push_back(jnt - joint_names.begin());
    }

    bool grippersIncomplete = false;
    std::vector<int> gripper_idx;
    for (int i = 0; i < gripper_joints.size(); ++i)
    {
        It jnt = std::find(joint_names.begin(), joint_names.end(), gripper_joints[i]);
        if (jnt == joint_names.end())
        {
            grippersIncomplete = true;
            break;
        }
        gripper_idx.push_back(jnt - joint_names.begin());
    }

    if (armIncomplete && grippersIncomplete)
    {
        // ROS_INFO("ArmComponentsNameManager::getJointIndices: Not all joint names present. List:");
        // for (std::vector<std::string>::const_iterator it=joint_names.begin(); it!=joint_names.end(); ++it) ROS_INFO("%s",it->c_str());
        return -1;
    }
    if (!armIncomplete)
    {
        idx.insert(idx.end(), arm_idx.begin(), arm_idx.end());
    }
    else
    {
        idx.insert(idx.end(), arm_joints.size(), -1);
    }
    if (!grippersIncomplete)
    {
        idx.insert(idx.end(), gripper_idx.begin(), gripper_idx.end());
    }
    else
    {
        idx.insert(idx.end(), gripper_joints.size(), -1);
    }

    return grippersIncomplete ? 1 : armIncomplete ? 2 : 0;
}


bool ArmComponentsNameManager::isGripper(const std::string& name) const
{
    std::vector<std::string>::const_iterator it;
    for (it = gripper_joints.begin(); it != gripper_joints.end(); ++it)
    {
        if (*it == name) return true;
    }
    return false;
}

int ArmComponentsNameManager::armJointNumber(const std::string& name) const
{
    for (int i = 0; i < arm_joints.size(); ++i)
    {
        const std::string& name_i = arm_joints[i];
        if (name_i == name) return i;
    }
    return -1;
}

int ArmComponentsNameManager::gripperJointNumber(const std::string& name) const
{
    for (int i = 0; i < gripper_joints.size(); ++i)
    {
        const std::string& name_i = gripper_joints[i];
        if (name_i == name) return i;
    }
    return -1;
}

int ArmComponentsNameManager::numArmJoints() const
{
    return arm_joints.size();
}

int ArmComponentsNameManager::numGripperJoints() const
{
    return gripper_joints.size();
}


void ArmComponentsNameManager::setValues(const std::string& _palm_link,
        const std::string& _effector_link,
        const std::vector<std::string>& _arm_joints, const std::vector<std::string>& _arm_links,
        const std::vector<std::string>& _gripper_joints, const std::vector<std::string>& _gripper_links,
        const std::vector<float>& _arm_joint_init, const std::vector<float>& _gripper_joint_init)
{
    palm_link = _palm_link;
    effector_link = _effector_link;
    arm_joints = _arm_joints;
    arm_links = _arm_links;
    gripper_joints = _gripper_joints;
    gripper_links = _gripper_links;
    arm_joint_init = _arm_joint_init;
    gripper_joint_init = _gripper_joint_init;
}


void ArmComponentsNameManager::setControllerNames(const std::vector<std::string>& controller_names, bool forArm, int type)
{
    if (forArm)
    {
        if (type == 0) arm_effort_controller_names = controller_names;
        else if (type == 1) arm_velocity_controller_names = controller_names;
        else if (type == 2) arm_position_controller_names = controller_names;
    }
    else
    {
        if (type == 0) gripper_effort_controller_names = controller_names;
        else if (type == 1) gripper_velocity_controller_names = controller_names;
        else if (type == 2) gripper_position_controller_names = controller_names;
    }
}
