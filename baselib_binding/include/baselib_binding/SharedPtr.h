#ifndef BASELIB_BINDING_SHAREDPTR_H
#define BASELIB_BINDING_SHAREDPTR_H

/**
   Macros for switching between shared pointer implementations.

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

#include <boost/shared_ptr.hpp>

#else  // use C++11 std

#include <memory>

#endif



namespace baselib_binding
{

#ifdef USE_BOOST

/*template <class T>
class shared_ptr: public boost::shared_ptr<T>
{ };*/

// make typedefs of this as follows:
// typedef baselib_binding::shared_ptr<CLASS>::type CLASSPtr;
template <class T>
struct shared_ptr
{
    typedef boost::shared_ptr<T> type;
};

#define baselib_binding_ns boost

#else  // use C++11 std

/*template <class T>
class shared_ptr: public std::shared_ptr<T>
{ };*/

// make typedefs of this as follows:
// typedef baselib_binding::shared_ptr<CLASS>::type CLASSPtr;
template <class T>
struct shared_ptr
{
    typedef std::shared_ptr<T> type;
};


#define baselib_binding_ns std

#endif

}  //namespace

#endif  // BASELIB_BINDING_SHAREDPTR_H
