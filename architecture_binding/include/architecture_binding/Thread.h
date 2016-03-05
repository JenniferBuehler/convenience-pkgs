#ifndef ARCHITECTURE_BINDING_THREAD_H
#define ARCHITECTURE_BINDING_THREAD_H

/**
   Macros for switching between thread implementations boost and std c++11.

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


#ifdef USE_BOOST

#include <boost/thread.hpp>

#else  // use c++11 std

#include <thread>
#include <mutex>
#include <chrono>

#endif



namespace architecture_binding
{

#ifdef USE_BOOST

typedef boost::thread thread;
typedef boost::mutex mutex;
typedef boost::recursive_mutex recursive_mutex;
typedef boost::condition_variable condition_variable;
typedef boost::posix_time::time_duration duration;

static duration get_duration_secs(double secs)
{
    return duration(0,0,secs,0);
}

// make typedefs of this as follows:
// typedef architecture_binding::unique_lock<CLASS>::type CLASSPtr;
template <class T>
struct unique_lock
{
    typedef boost::unique_lock<T> type;
};

#define SLEEP(secs) { boost::this_thread::sleep(boost::posix_time::milliseconds(secs*1000)); }


// ----------------------------------------------
#else  // use c++11 std
// ----------------------------------------------


typedef std::thread thread;
typedef std::mutex mutex;
typedef std::recursive_mutex recursive_mutex;
typedef std::condition_variable condition_variable;
typedef std::chrono::duration;

static duration get_duration_secs(double secs)
{
    return std::chrono::seconds(secs);
}

// make typedefs of this as follows:
// typedef architecture_binding::unique_lock<CLASS>::type CLASSPtr;
template <class T>
struct unique_lock
{
    typedef std::unique_lock<T> type;
};

#define SLEEP(secs) \
{ \
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(std::floor(secs*1000))));\
}

#endif

}  // namespace
#endif  // ARCHITECTURE_BINDING_THREAD_H
