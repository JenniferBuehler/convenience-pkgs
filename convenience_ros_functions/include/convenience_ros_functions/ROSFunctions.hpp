
template<typename ROSMessage>
bool ROSFunctions::saveToFile(const ROSMessage& msg, const std::string& filename, bool asBinary)
{

    std::ios_base::openmode mode;
    if (asBinary) mode = std::ios::out | std::ios::binary;
    else mode = std::ios::out;

    std::ofstream ofs(filename.c_str(), mode);

    if (!ofs.is_open())
    {
        ROS_ERROR("File %s cannot be opened.", filename.c_str());
        return false;
    }

    if (asBinary)
    {
        uint32_t serial_size = ros::serialization::serializationLength(msg);
        boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
        ros::serialization::OStream ostream(obuffer.get(), serial_size);
        ros::serialization::serialize(ostream, msg);
        ofs.write((char*) obuffer.get(), serial_size);
    }
    else
    {
        ofs<<msg; 
    }

    ofs.close();
    return true;
}

template<typename ROSMessage>
bool ROSFunctions::readFromFile(const std::string& filename, ROSMessage& msg, bool isBinary)
{

    std::ios_base::openmode mode;
    if (isBinary) mode = std::ios::in | std::ios::binary;
    else mode = std::ios::in;

    std::ifstream ifs(filename.c_str(), mode);

    if (!ifs.is_open())
    {
        ROS_ERROR("File %s cannot be opened.", filename.c_str());
        return false;
    }

    ifs.seekg(0, std::ios::end);
    std::streampos end = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    std::streampos begin = ifs.tellg();

    uint32_t file_size = end - begin;

    boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);

    ifs.read((char*) ibuffer.get(), file_size);

    ros::serialization::IStream istream(ibuffer.get(), file_size);

    ros::serialization::deserialize(istream, msg);

    ifs.close();

    return true;
}


template<typename ROSMessage>
bool ROSFunctions::saveToBagFile(const std::vector<ROSMessage>& msgs, const std::string& filename)
{
    rosbag::Bag bag(filename, rosbag::bagmode::Write);
    for (typename std::vector<ROSMessage>::const_iterator m = msgs.begin(); m != msgs.end(); ++m)
    {
        bag.write("filesave", ros::Time::now(), *m);
    }
    bag.close();
    return true;
}

template<typename ROSMessage>
bool ROSFunctions::readFromBagFile(const std::string& filename, std::vector<ROSMessage>& msgs)
{
    rosbag::Bag bag(filename, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery("filesave"));
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        typename ROSMessage::ConstPtr msg = m.instantiate<ROSMessage>();
        if (msg != NULL)
        {
            msgs.push_back(*msg);
        }
    }
    bag.close();
    return true;
}

template<typename MJointState>
bool ROSFunctions::getPoseFromVirtualJointState(const std::vector<std::string>& joint_names, const MJointState& virtualJointState,
        const std::string& virtualJointName, geometry_msgs::PoseStamped& robotPose)
{
    if (virtualJointName.empty()) return false;

    std::vector<std::string>::const_iterator it = std::find(joint_names.begin(), joint_names.end(), virtualJointName);
    if (it == joint_names.end()) return false;

    int idx = (it - joint_names.begin());
    geometry_msgs::Transform t = virtualJointState.transforms[idx];
    robotPose.pose.position.x = t.translation.x;
    robotPose.pose.position.y = t.translation.y;
    robotPose.pose.position.z = t.translation.z;
    robotPose.pose.orientation = t.rotation;
    return true;
}

template<typename GH, typename Res>
void ROSFunctions::mapToGoalHandle(const actionlib::SimpleClientGoalState& state, GH& goalHandle,
                                   const Res& res, const std::string& s)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        goalHandle.setSucceeded(res, s);
    }
    else if (actionlib::SimpleClientGoalState::REJECTED)
    {
        goalHandle.setRejected(res, s);
    }
    else if ((state == actionlib::SimpleClientGoalState::RECALLED) ||
             (state == actionlib::SimpleClientGoalState::PREEMPTED))
    {
        goalHandle.setCanceled(res, s);
    }
    else if ((state == actionlib::SimpleClientGoalState::ACTIVE) ||
             (state == actionlib::SimpleClientGoalState::PENDING))
    {
        goalHandle.setAccepted();
    }
    else if ((state == actionlib::SimpleClientGoalState::LOST) ||
             (state == actionlib::SimpleClientGoalState::ABORTED))
    {
        goalHandle.setAborted(res, s);
    }
    else
    {
        ROS_ERROR("ROSFunctions: Unknown goal handle");
    }
}

template<typename GH, typename Res>
void ROSFunctions::effectOnGoalHandle(const actionlib::SimpleClientGoalState& state, GH& goalHandle,
                                      const Res& res, const std::string& s)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        goalHandle.setSucceeded(res, s);
    }
    else if (actionlib::SimpleClientGoalState::REJECTED)
    {
        goalHandle.setAborted(res, s);
    }
    else if ((state == actionlib::SimpleClientGoalState::RECALLED) ||
             (state == actionlib::SimpleClientGoalState::PREEMPTED))
    {
        goalHandle.setCanceled(res, s);
    }
    else if ((state == actionlib::SimpleClientGoalState::ACTIVE) ||
             (state == actionlib::SimpleClientGoalState::PENDING))
    {
        goalHandle.setAccepted();
    }
    else if ((state == actionlib::SimpleClientGoalState::LOST) ||
             (state == actionlib::SimpleClientGoalState::ABORTED))
    {
        goalHandle.setAborted(res, s);
    }
    else
    {
        ROS_ERROR("ROSFunctions: Unknown goal handle");
    }
}

template<typename GH>
void ROSFunctions::effectOnGoalHandle(const actionlib::SimpleClientGoalState& state, GH& goalHandle)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        goalHandle.setSucceeded();
    }
    else if (actionlib::SimpleClientGoalState::REJECTED)
    {
        goalHandle.setAborted();
    }
    else if ((state == actionlib::SimpleClientGoalState::RECALLED) ||
             (state == actionlib::SimpleClientGoalState::PREEMPTED))
    {
        goalHandle.setCanceled();
    }
    else if ((state == actionlib::SimpleClientGoalState::ACTIVE) ||
             (state == actionlib::SimpleClientGoalState::PENDING))
    {
        goalHandle.setAccepted();
    }
    else if ((state == actionlib::SimpleClientGoalState::LOST) ||
             (state == actionlib::SimpleClientGoalState::ABORTED))
    {
        goalHandle.setAborted();
    }
    else
    {
        ROS_ERROR("ROSFunctions: Unknown goal handle");
    }
}

