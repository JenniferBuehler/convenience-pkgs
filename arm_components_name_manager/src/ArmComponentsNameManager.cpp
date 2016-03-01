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
    arm_joints = getDefaultArmJoints();
    arm_links = getDefaultArmLinks();
    arm_joint_init = getDefaultArmJointsInitPose();
    gripper_joints = getDefaultGripperJoints();
    gripper_links = getDefaultGripperLinks();
    gripper_joint_init = getDefaultGripperJointsInitPose();
    initWithDefaults = true;
    return true;
}

int ArmComponentsNameManager::loadParameters()
{
    bool allControllersSpecified = true;
    int noSpec = 0;
    int numSpecs = 0;

    ros::NodeHandle robot_nh(robot_namespace);
    ROS_INFO_STREAM("ArmComponentsNameManager reading parameters from namespace: " << robot_nh.getNamespace());

    // ROS_INFO_STREAM("Reading palm_link:");
    robot_nh.getParam("palm_link", palm_link);
    if (palm_link.empty())
    {
        ++noSpec;
        ROS_ERROR("Parameter palm_link should be specified");
    }
    ++numSpecs;

    // --- arm parameters

    // ROS_INFO_STREAM("Reading arm_joints:");
    robot_nh.getParam("arm_joints", arm_joints);
    if (arm_joints.empty())
    {
        ++noSpec;
        ROS_ERROR("Parameter arm_joints should be specified as an array");
    }
    ++numSpecs;
    // for (int i=0; i < arm_joints.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_joints[i]);}

    // ROS_INFO_STREAM("Reading arm_joint_init:");
    robot_nh.getParam("arm_joint_init", arm_joint_init);
    if (arm_joint_init.empty())
    {
        ++noSpec;
        ROS_ERROR("Parameter arm_joint_init should be specified as an array");
    }
    ++numSpecs;
    // for (int i=0; i < arm_joint_init.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_joint_init[i]);}


    // ROS_INFO_STREAM("Reading arm_links:");
    robot_nh.getParam("arm_links", arm_links);
    if (arm_links.empty())
    {
        ++noSpec;
        ROS_ERROR("Parameter arm_links should be specified as an array");
    }
    ++numSpecs;
    // for (int i=0; i < arm_links.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_links[i]);}

    // controllers

    // ROS_INFO_STREAM("Reading arm_position_controller_names:");
    robot_nh.getParam("arm_position_controller_names", arm_position_controller_names);
    if (arm_position_controller_names.empty())
    {
        allControllersSpecified = false;
        ROS_INFO("INFO: Parameter arm_position_controller_names has not been specified");
    }
    // for (int i=0; i < arm_position_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_position_controller_names[i]);}

    // ROS_INFO_STREAM("Reading arm_velocity_controller_names:");
    robot_nh.getParam("arm_velocity_controller_names", arm_velocity_controller_names);
    if (arm_velocity_controller_names.empty())
    {
        allControllersSpecified = false;
        ROS_INFO("INFO: Parameter arm_velocity_controller_names has not been specified");
    }
    // for (int i=0; i < arm_velocity_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_velocity_controller_names[i]);}

    // ROS_INFO_STREAM("Reading arm_effort_controller_names:");
    robot_nh.getParam("arm_effort_controller_names", arm_effort_controller_names);
    if (arm_effort_controller_names.empty())
    {
        allControllersSpecified = false;
        ROS_INFO("INFO: Parameter arm_effort_controller_names has not been specified");
    }
    // for (int i=0; i < arm_effort_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_effort_controller_names[i]);}



    // --- gripper parameters

    // ROS_INFO_STREAM("Reading gripper_joints:");
    robot_nh.getParam("gripper_joints", gripper_joints);
    if (gripper_joints.empty())
    {
        ++noSpec;
        ROS_ERROR("Parameter gripper_joints should be specified as an array");
    }
    ++numSpecs;
    // for (int i=0; i < gripper_joints.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_joints[i]);}

    // ROS_INFO_STREAM("Reading gripper_joint_init:");
    robot_nh.getParam("gripper_joint_init", gripper_joint_init);
    if (gripper_joint_init.empty())
    {
        ++noSpec;
        ROS_ERROR("Parameter gripper_joint_init should be specified as an array");
    }
    ++numSpecs;
    // for (int i=0; i < gripper_joint_init.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_joint_init[i]);}


    // ROS_INFO_STREAM("Reading gripper_links:");
    robot_nh.getParam("gripper_links", gripper_links);
    if (gripper_links.empty())
    {
        ++noSpec;
        ROS_ERROR("Parameter gripper_links should be specified as an array");
    }
    ++numSpecs;
    // for (int i=0; i < gripper_links.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_links[i]);}

    // controllers

    // ROS_INFO_STREAM("Reading gripper_position_controller_names:");
    robot_nh.getParam("gripper_position_controller_names", gripper_position_controller_names);
    if (gripper_position_controller_names.empty())
    {
        allControllersSpecified = false;
        ROS_INFO("INFO: Parameter gripper_position_controller_names has not been specified");
    }
    // for (int i=0; i < gripper_position_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_position_controller_names[i]);}

    // ROS_INFO_STREAM("Reading gripper_velocity_controller_names:");
    robot_nh.getParam("gripper_velocity_controller_names", gripper_velocity_controller_names);
    if (gripper_velocity_controller_names.empty())
    {
        allControllersSpecified = false;
        ROS_INFO("INFO: Parameter gripper_velocity_controller_names has not been specified");
    }
    // for (int i=0; i < gripper_velocity_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_velocity_controller_names[i]);}

    // ROS_INFO_STREAM("Reading gripper_effort_controller_names:");
    robot_nh.getParam("gripper_effort_controller_names", gripper_effort_controller_names);
    if (gripper_effort_controller_names.empty())
    {
        allControllersSpecified = false;
        ROS_INFO("INFO: Parameter gripper_effort_controller_names has not been specified");
    }
    // for (int i=0; i < gripper_effort_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_effort_controller_names[i]);}

    initParamCode = 0;
    if (noSpec == numSpecs) initParamCode =  -1;
    if ((noSpec == 0) && allControllersSpecified) initParamCode =  2;
    else if (noSpec == 0) initParamCode =  1;
    return initParamCode;
}


bool ArmComponentsNameManager::waitToLoadParameters(int sufficientSuccessCode, float maxWait)
{
    float checkSteps = 0.1;
    float timeWaited = 0;
    while (timeWaited < maxWait)
    {
        int loadParamRet = loadParameters();
        if (loadParamRet >= sufficientSuccessCode)
            return true;
        ros::Duration(checkSteps).sleep();
        timeWaited += checkSteps;
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

void ArmComponentsNameManager::initJointState(sensor_msgs::JointState& js, bool withGripper, const std::vector<float> * init_poses) const
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
}


int ArmComponentsNameManager::getJointIndices(const std::vector<std::string>& joint_names, std::vector<int>& idx)
{
    typedef std::vector<std::string>::const_iterator It;

    It jnt1 = std::find(joint_names.begin(), joint_names.end(), arm_joints[0]);
    It jnt2 = std::find(joint_names.begin(), joint_names.end(), arm_joints[1]);
    It jnt3 = std::find(joint_names.begin(), joint_names.end(), arm_joints[2]);
    It jnt4 = std::find(joint_names.begin(), joint_names.end(), arm_joints[3]);
    It jnt5 = std::find(joint_names.begin(), joint_names.end(), arm_joints[4]);
    It jnt6 = std::find(joint_names.begin(), joint_names.end(), arm_joints[5]);
    It fjnt1 = std::find(joint_names.begin(), joint_names.end(), gripper_joints[0]);
    It fjnt2 = std::find(joint_names.begin(), joint_names.end(), gripper_joints[1]);
    It fjnt3 = std::find(joint_names.begin(), joint_names.end(), gripper_joints[2]);

    bool armIncomplete = (jnt1 == joint_names.end()) ||
                         (jnt2 == joint_names.end()) ||
                         (jnt3 == joint_names.end()) ||
                         (jnt4 == joint_names.end()) ||
                         (jnt5 == joint_names.end()) ||
                         (jnt6 == joint_names.end());
    bool grippersIncomplete = (fjnt1 == joint_names.end()) || (fjnt2 == joint_names.end()) || (fjnt3 == joint_names.end());

    if (armIncomplete && grippersIncomplete)
    {
        // ROS_INFO("ArmComponentsNameManager::getJointIndices: Not all joint names present in trajectory. List:");
        // for (std::vector<std::string>::const_iterator it=joint_names.begin(); it!=joint_names.end(); ++it) ROS_INFO("%s",it->c_str());
        return -1;
    }
    if (!armIncomplete)
    {
        idx.push_back(jnt1 - joint_names.begin());
        idx.push_back(jnt2 - joint_names.begin());
        idx.push_back(jnt3 - joint_names.begin());
        idx.push_back(jnt4 - joint_names.begin());
        idx.push_back(jnt5 - joint_names.begin());
        idx.push_back(jnt6 - joint_names.begin());
    }
    else
    {
        idx.insert(idx.end(), 6, -1);
    }
    if (!grippersIncomplete)
    {
        idx.push_back(fjnt1 - joint_names.begin());
        idx.push_back(fjnt2 - joint_names.begin());
        idx.push_back(fjnt3 - joint_names.begin());
    }
    else
    {
        idx.insert(idx.end(), 3, -1);
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

/*
double ArmComponentsNameManager::capToPI(const double value)
{
    static const double pi_2 = 2.0 * M_PI;
    double v = value;
    if (v <= -M_PI || v > M_PI)
    {
        v = fmod(v, pi_2);
        if (v <= -M_PI)
            v += pi_2;
        else if (v > M_PI)
            v -= pi_2;
    }
    return v;
}

double ArmComponentsNameManager::limitsToTwoPI(const double value, const double lowLimit, const double highLimit)
{
    double ret = value;
    if (value > highLimit) ret = value - 2*M_PI;
    if (value < lowLimit) ret = value + 2*M_PI;
    return ret;
}



double ArmComponentsNameManager::angleDistance(const double _f1, const double _f2)
{
    double f1 = capToPI(_f1);
    double f2 = capToPI(_f2);
    double diff = f2 - f1;
    diff = capToPI(diff);
    return diff;
}
*/


int ArmComponentsNameManager::numArmJoints() const
{
    return arm_joints.size();
}

int ArmComponentsNameManager::numGripperJoints() const
{
    return gripper_joints.size();
}


void ArmComponentsNameManager::setValues(const std::string& _palm_link,
        const std::vector<std::string>& _arm_joints, const std::vector<std::string>& _arm_links,
        const std::vector<std::string>& _gripper_joints, const std::vector<std::string>& _gripper_links,
        const std::vector<float>& _arm_joint_init, const std::vector<float>& _gripper_joint_init)
{
    palm_link = _palm_link;
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
