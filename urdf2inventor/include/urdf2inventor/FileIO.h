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

#ifndef URDF2INVENTOR_FILEIO_H
#define URDF2INVENTOR_FILEIO_H
// Copyright Jennifer Buehler

//-----------------------------------------------------
#include <urdf/model.h>
#include <urdf2inventor/ConversionResult.h>
#include <architecture_binding/SharedPtr.h>

#include <iostream>
#include <string>
#include <map>
#include <vector>

namespace urdf2inventor
{

/**
 * \brief Reads and writes URDF and GraspIt! files to disk.
 * 
 *  Parameter MeshFormat: Datatype to use for the meshes (e.g. a string for XML).
 * \author Jennifer Buehler
 * \date March 2016
 */
template<typename MeshFormat=std::string>
class FileIO
{
public:
    typedef ConversionResult<MeshFormat> ConversionResultT;
    typedef typename architecture_binding::shared_ptr<ConversionResultT>::type ConversionResultPtr;

    /**
     * \param _outputDir directory where to save the files.
     */
    explicit FileIO(const std::string& _outputDir):
        outputDir(_outputDir){}

    virtual ~FileIO()
    {
    }

    /**
     * Initializes the directory, then writes the meshes.
     */    
    bool write(const ConversionResultPtr& data) const;

protected:
    /**
     * Initializes the output directory. Will also call initOutputDirImpl() after
     * creating the directory \e outputDir.
     */
    bool initOutputDir() const;

    bool writeMeshFiles(const std::map<std::string, MeshFormat>& meshes,
                        const std::string& MESH_OUTPUT_EXTENSION,
                        const std::string& MESH_OUTPUT_DIRECTORY_NAME) const;

    /**
     * Called from initOutputDir(), can be used by subclassees
     */
    virtual bool initOutputDirImpl() const { return true; };
    
    /**
     * Called from write(ConversionResultPtr&), after initOutputDir() has been called and
     * the meshes (in base class of \e data) has been written with writeMeshFiles().
     * Can be used by subclasses to write other things belonging to the result.
     */
    virtual bool writeImpl(const ConversionResultPtr& data) const { return true; }

    std::string outputDir;
};
    
#include <urdf2inventor/FileIO.hpp>

}  //  namespace urdf2inventor
#endif   // URDF2INVENTOR_FILEIO_H
