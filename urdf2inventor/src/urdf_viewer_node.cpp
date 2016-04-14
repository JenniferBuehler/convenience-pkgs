/**
    Copyright (C) 2015 Jennifer Buehler

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

#include <urdf2inventor/Urdf2Inventor.h>
#include <urdf2inventor/InventorViewer.h>
#include <string>

using urdf2inventor::viewer::InventorViewer;
using urdf2inventor::Urdf2Inventor;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " <input-file> [--iv] [<from-link>]" << std::endl;
        std::cout <<" If --iv is specified, the file is assumed to be an inventor file, otherwise a URDF file." <<std::endl;
        std::cout <<" if <from-link> is specified (only supported if not using --iv), the URDF is converted from this link down." <<std::endl;
        return 0;
    }

    bool isURDF = true;

    std::string inputFile = argv[1];
    std::string fromLink;
 
    if (argc > 2)
    {
        std::string arg(argv[2]);
        if (arg=="--iv") isURDF=false;
        else if (arg!="root") fromLink=arg;
    } 

    /**
     * TODO: For some reason I haven't yet further investigated, view.init() has to be called after
     * loadAndGetAsInventor(), or it won't work. Find out why, and fix it.
     * It has probably to do with the calls of SoDB::init involved by ivcon as well.
     */

    bool success = true;
    Urdf2Inventor converter;
    InventorViewer view;
    if (isURDF)
    {
        std::cout<<"Converting model from file "<<inputFile<<"..."<<std::endl;
        if (!fromLink.empty()) std::cout<<"Staring from link "<<fromLink<<std::endl;

        if (!converter.loadModelFromFile(inputFile))
        {
            std::cerr<<"Could not load file "<<inputFile<<std::endl;
            return 0;
        }
        converter.printJointNames(fromLink);

        // TODO: For texting, fixed links can be joined and axes rotated.
        // This can be parameterized at some point, for now it's only used for testing.
        bool joinFixedLinks = true;
        if (joinFixedLinks && !converter.joinFixedLinks(fromLink))
        {
            std::cerr<<"Could not join fixed links"<<std::endl;
            return 0;
        }
        
        bool rotateAxes = true;
        Eigen::Vector3d axis(0,0,1);
        if (rotateAxes && !converter.allRotationsToAxis(fromLink, axis))
        {
            std::cerr<<"Could not rotate axes"<<std::endl;
            return 0;
        }
    
        SoNode * node = converter.getAsInventor(fromLink, false);
        if (!node)
        {
            std::cout<<"ERROR: Could not get inventor node"<<std::endl;
            success = false;
        }else{
            std::cout<<"Model converted, now loading into viewer..."<<std::endl;
            view.init("WindowName");
            view.loadModel(node);
        }
    }
    else
    {
        view.init("WindowName");
        view.loadModel(inputFile);
    }
    if (success)  view.runViewer();

    bool deleteOutputRedirect = true;
    converter.cleanup(deleteOutputRedirect);
    return 0;
}


