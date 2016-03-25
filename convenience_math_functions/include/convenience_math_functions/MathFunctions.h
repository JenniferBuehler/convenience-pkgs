#ifndef CONVENIENCE_MATH_FUNCTIONS_MATHFUNCTIONS_H
#define CONVENIENCE_MATH_FUNCTIONS_MATHFUNCTIONS_H

#include <eigen_conversions/eigen_msg.h>
#include <fstream>

namespace convenience_math_functions
{

/**
 * A collection of helper math functions
 *
 * \author Jennifer Buehler
 */
class MathFunctions
{

public:
    /**
     */
    MathFunctions() {}

    ~MathFunctions()
    {
    }

    /**
     * enforces the bound of the angle to be between -PI and PI
     * this is a useful helper for lots of places because the
     * native jaco joint angles may go beyond PI.
     */
    static double capToPI(const double value);

    template<typename Flt>
    static void capToPI(std::vector<Flt>& v)
    {
        for (typename std::vector<Flt>::iterator it = v.begin(); it != v.end(); ++it)
        {
            *it = capToPI(*it);
        }
    }

    /**
     * Transforms a value between -PI and PI to a value between -2*PI and +2*PI such
     * that the joint limits are respected. It is assumed all values i have to be between
     * between lower bound l and higher bound h, and always l <= i <= h.
     * So e.g. l=-3.9 .. h=0.8 instead of
     * l=2.38 .. h=0.8. So lowLimit can *always* be smaller than highlimit, and there
     * never is a jump from PI to -PI or the other way round.
     */
    static double limitsToTwoPI(const double value, const double lowLimit, const double highLimit);

    /**
     * Returns shortest distace between two angles (in rad), specifically
     * when going from \e _f2 to \e _f1
     */
    static double angleDistance(const double _f1, const double _f2);

    static double quatAngularDistance(const geometry_msgs::Quaternion& _q1, const geometry_msgs::Quaternion& _q2);

    static double quatAngularDistance(const Eigen::Quaterniond& _q1, const Eigen::Quaterniond& _q2);

    static double vecAngularDistance(const Eigen::Vector3d& _v1, const Eigen::Vector3d& _v2);

    static Eigen::Quaterniond getRotationFromTo(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);

    static bool equalFlt(float first, float second, float tolerance);
    /**
     * equalFlt(float, float, float) returns true for all in vector
     */
    static bool equalFloats(const std::vector<float>& first, const std::vector<float>& second, float tolerance);
};
}
#endif // CONVENIENCE_MATH_FUNCTIONS_MATHFUNCTIONS_H
