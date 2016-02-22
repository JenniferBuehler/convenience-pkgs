/**
    Reads in an inventor data file, and uses the SoQT Example Viewer simple viewer to view the data.

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
 */



#include <Inventor/SoDB.h>
#include <Inventor/SoInput.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoMatrixTransform.h>

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>

/**
 * Adds a SoNode as a child using the given transform.
 */
SoSeparator * combine(SoNode* parent, SoNode * addAsChild, SoTransform * trans)
{
    SoSeparator * sep = dynamic_cast<SoSeparator*>(parent);
    if (!sep)
    {
        printf("Error: parent is not a separator");
        return NULL;
    }

    SoSeparator * sepChild = dynamic_cast<SoSeparator*>(addAsChild);
    if (!sepChild)
    {
        printf("Error: child is not a separator");
        return NULL;
    }

    SoSeparator * transNode = new SoSeparator();
    transNode->addChild(trans);
    transNode->addChild(sepChild);

    sep->addChild(transNode);
    return sep;
}


int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Usage: %s file.iv\n", argv[0]);
        return 1;
    }

    // Initialize Inventor and Xt
    QWidget * myWindow = SoQt::init(argv[0]);

    // Read file in
    SoInput in;
    SoNode  *scene;
    if (! in.openFile(argv[1]))
        return 1;
    if (! SoDB::read(&in, scene) || scene == NULL)
        return 1;

    if (argc > 2)
    {
        printf("Adding another file\n");
        SoNode  *child;
        SoInput inc;
        if (! inc.openFile(argv[2]))
            return 1;
        if (! SoDB::read(&inc, child) || child == NULL)
            return 1;

        SoTransform * trans = new SoTransform();
        trans->translation.setValue(0, 0, 50);
        trans->rotation.setValue(0, 0, 0, 1); //quaternion
        scene = combine(scene, child, trans);
        if (!scene)
        {
            printf("Error: failed combining\n");
            return 1;
        }
    }

    SoQtExaminerViewer * viewer = new SoQtExaminerViewer(myWindow);
    viewer->setSceneGraph(scene);
    viewer->show();

    // create and show the viewer
    /*SimpleViewer *myViewer = new SimpleViewer(myWindow);
    myViewer->setSceneGraph(scene);
    myViewer->setTitle("Custom Viewer");
    myViewer->show();*/

    SoQt::show(myWindow);
    SoQt::mainLoop();
}
