#ifndef LOGGER_BINDING_ROS_LOGBINDINGROS_H
#define LOGGER_BINDING_ROS_LOGBINDINGROS_H
/**
   Logging class implementation for ROS

   Copyright (C) 2016 Jennifer Buehler

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

#include <logger_binding/LogBinding.h>
#include <ros/ros.h>
#include <string>

namespace logger_binding
{

/**
 * \brief Class to bind to the ROS message printing.
 * \author Jennifer Buehler
 * \date January 2016
 */

class ROSLog: public Log
{
protected:
    virtual void implPrint(const std::stringstream& str)
    {
        ROS_INFO_STREAM(str.str());
    }
    virtual void implPrintError(const std::stringstream& str)
    {
        ROS_ERROR_STREAM(str.str());
    }
    virtual void implPrintWarn(const std::stringstream& str)
    {
        ROS_WARN_STREAM(str.str());
    }

    virtual void implPrint(const char* str)
    {
        ROS_INFO_STREAM(str);
    }
    virtual void implPrintError(const char* str)
    {
        ROS_ERROR_STREAM(str);
    }
    virtual void implPrintWarn(const char* str)
    {
        ROS_WARN_STREAM(str);
    }

    virtual void printNewLine(bool errorStream)
    {
    }
};

}  // namespace logger_binding


#define PRINT_INIT_ROS() \
{\
    if (logger_binding::Log::Singleton) \
    { \
        std::cerr << "Singleton already set, overwriting!" << std::endl;\
    } \
    logger_binding::Log::Singleton = logger_binding::Log::LogPtr(new logger_binding::ROSLog()); \
}

#endif  // LOGGER_BINDING_ROS_LOGBINDINGROS_H
