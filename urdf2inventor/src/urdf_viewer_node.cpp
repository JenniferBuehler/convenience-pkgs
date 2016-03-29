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
    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " <input-file>.urdf" << std::endl;
        return 1;
    }

    std::string inputFile = argv[1];

    Urdf2Inventor converter;
    SoNode * node = converter.loadAndGetAsInventor(inputFile);

    InventorViewer view;
    view.init("WindowName");
    view.loadModel(node);
    view.runViewer();

    bool deleteOutputRedirect = true;
    converter.cleanup(deleteOutputRedirect);
    return 0;
}


