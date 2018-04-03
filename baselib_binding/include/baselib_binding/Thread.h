#ifndef BASELIBITECTURE_BINDING_THREAD_H
#define BASELIBITECTURE_BINDING_THREAD_H

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
#if defined(USE_BOOST) and defined (USE_C11)
    "ERROR: Inconsistent use of boost and C++11 at the same time"
#endif

#if !defined(USE_BOOST) and !defined (USE_C11)
    "ERROR: You need to have defined either USE_BOOST or USE_C11, have you included the definitions for this package in your build file (eg cmake)?"
#endif



#ifdef USE_BOOST

#include <boost/thread.hpp>

#else  // use c++11 std

#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>

#endif



namespace baselib_binding
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
// typedef baselib_binding::unique_lock<CLASS>::type CLASSPtr;
template <class T>
struct unique_lock
{
    typedef boost::unique_lock<T> type;
};

// library-specific all to sleep for secs seconds
#define SLEEP(secs) { boost::this_thread::sleep(boost::posix_time::milliseconds(secs*1000)); }

// library-specific call to condition variable's timed wait. Returns a boolean whether the wait was successful (no timeout)
#define COND_WAIT(cond_var, lock_guard, timeout) cond_var.timed_wait(lock_guard, baselib_binding::get_duration_secs(timeout))

// ----------------------------------------------
#else  // use c++11 std
// ----------------------------------------------


typedef std::thread thread;
typedef std::mutex mutex;
typedef std::recursive_mutex recursive_mutex;
typedef std::condition_variable condition_variable;
typedef std::chrono::duration<double, std::ratio<1> > duration;

static duration get_duration_secs(double secs)
{
    return duration(secs);
}

// make typedefs of this as follows:
// typedef baselib_binding::unique_lock<CLASS>::type CLASSPtr;
template <class T>
struct unique_lock
{
    typedef std::unique_lock<T> type;
};


// library-specific all to sleep for secs seconds
#define SLEEP(secs) \
{ \
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(std::floor(secs*1000))));\
}

// library-specific call to condition variable's timed wait. Returns a boolean whether the wait was successful (no timeout)
#define COND_WAIT(cond_var, lock_guard, timeout) cond_var.wait_for(lock_guard, baselib_binding::get_duration_secs(timeout)) == std::cv_status::no_timeout

#endif

}  // namespace
#endif  // BASELIBITECTURE_BINDING_THREAD_H
