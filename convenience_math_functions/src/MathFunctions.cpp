#include <convenience_math_functions/MathFunctions.h>
#include <algorithm>
#include <eigen_conversions/eigen_msg.h>

using convenience_math_functions::MathFunctions;

double MathFunctions::capToPI(const double value)
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

double MathFunctions::limitsToTwoPI(const double value, const double lowLimit, const double highLimit)
{
    double ret = value;
    if (value > highLimit) ret = value - 2 * M_PI;
    if (value < lowLimit) ret = value + 2 * M_PI;
    return ret;
}


Eigen::Quaterniond MathFunctions::getRotationFromTo(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2)
{
    Eigen::Quaterniond ret = q2 * q1.inverse();
    ret.normalize();
    return ret;
}


double MathFunctions::angleDistance(const double _f1, const double _f2)
{
    double f1 = capToPI(_f1);
    double f2 = capToPI(_f2);
    double diff = f2 - f1;
    diff = capToPI(diff);
    /*ROS_INFO("Cap1 %f to %f",_f1,f1);
    ROS_INFO("Cap2 %f to %f",_f2,f2);
    ROS_INFO("DIFF %f",diff);*/
    /*      if (diff > M_PI) diff=M_PI-diff;
        else if (diff < -M_PI) diff=-M_PI+diff;
        ROS_INFO("DIFF2 %f",diff);*/
    return diff;
}


double MathFunctions::quatAngularDistance(const geometry_msgs::Quaternion& _q1, const geometry_msgs::Quaternion& _q2)
{
    Eigen::Quaterniond q1_in, q2_in;
    tf::quaternionMsgToEigen(_q1, q1_in);
    tf::quaternionMsgToEigen(_q2, q2_in);
    return quatAngularDistance(q1_in, q2_in);
}


double MathFunctions::quatAngularDistance(const Eigen::Quaterniond& _q1, const Eigen::Quaterniond& _q2)
{
    if (_q1.isApprox(_q2, 0.001)) return 0;
    Eigen::Quaterniond q1 = _q1;
    Eigen::Quaterniond q2 = _q2;
    q1.normalize();
    q2.normalize();
    return q1.angularDistance(q2);
}

double MathFunctions::vecAngularDistance(const Eigen::Vector3d& _v1, const Eigen::Vector3d& _v2)
{
    Eigen::Vector3d v1 = _v1;
    Eigen::Vector3d v2 = _v2;
    v1.normalize();
    v2.normalize();
    return acos(v1.dot(v2));
}


bool MathFunctions::equalFlt(float first, float second, float tolerance)
{
    return ((first <= second + tolerance) && (first >= second - tolerance));
}

bool MathFunctions::equalFloats(const std::vector<float>& first, const std::vector<float>& second, float tolerance)
{
    if (first.size() != second.size()) return false;
    for (int i = 0; i < first.size(); ++i)
        if (!MathFunctions::equalFlt(first[i], second[i], tolerance)) return false;
    return true;
}
