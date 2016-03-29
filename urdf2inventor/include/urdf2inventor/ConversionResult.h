/**
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
**/

#ifndef URDF2INVENTOR_CONVERSIONRESULT_H
#define URDF2INVENTOR_CONVERSIONRESULT_H
// Copyright Jennifer Buehler

#include <string>
#include <map>

namespace urdf2inventor
{

/**
 * \brief Encapsulates all result fields for a conversion
 * \author Jennifer Buehler
 * \date November 2015
 */
template<typename MeshFormat>
class ConversionResult
{
public:
    explicit ConversionResult(const std::string& _meshOutputExtension, const std::string& _meshOutputDirectoryName):
        meshOutputExtension(_meshOutputExtension),
        meshOutputDirectoryName(_meshOutputDirectoryName),
        success(false) {}
    ConversionResult(const ConversionResult& o):
        meshes(o.meshes),
        meshOutputExtension(o.meshOutputExtension),
        meshOutputDirectoryName(o.meshOutputDirectoryName),
        success(o.success) {}

    virtual ~ConversionResult() {}

    // the resulting meshes (inventor files), indexed by the link name
    std::map<std::string, MeshFormat> meshes;

    // the extension to use for mesh files (e.g. ".iv" for inventor)
    std::string meshOutputExtension;

    std::string meshOutputDirectoryName;
    
    bool success;

private:
    ConversionResult():
        success(false) {}
};

}  //  namespace urdf2inventor
#endif   // URDF2INVENTOR_CONVERSIONRESULT_H
