#include <urdf2inventor/IVHelpers.h>
#include <iostream>

SoTransform * urdf2inventor::getTransform(const urdf2inventor::EigenTransform& eTrans)
{
    SoTransform * transform = new SoTransform();

    SoSFVec3f translation;
    translation.setValue(eTrans.translation().x(), eTrans.translation().y(), eTrans.translation().z());
    transform->translation = translation;

    SoSFRotation rotation;
    Eigen::Quaterniond vQuat(eTrans.rotation());
    rotation.setValue(vQuat.x(), vQuat.y(), vQuat.z(), vQuat.w());
    transform->rotation = rotation;
    return transform;
}

SoSeparator * urdf2inventor::addSubNode(SoNode * addAsChild, SoNode* parent, const urdf2inventor::EigenTransform& eTrans)
{
    SoTransform * transform = getTransform(eTrans);
    return urdf2inventor::addSubNode(addAsChild, parent, transform);
}

SoSeparator * urdf2inventor::addSubNode(SoNode * addAsChild, SoNode* parent, SoTransform * trans)
{
    SoSeparator * sep = dynamic_cast<SoSeparator*>(parent);
    if (!sep)
    {
        std::cerr<<"parent is not a separator"<<std::endl;
        return NULL;
    }

    SoSeparator * sepChild = dynamic_cast<SoSeparator*>(addAsChild);
    if (!sepChild)
    {
        std::cerr<<"child is not a separator"<<std::endl;
        return NULL;
    }

    // ROS_WARN_STREAM("######### Adding transform "<<trans->translation<<", "<<trans->rotation);

    SoSeparator * transNode = new SoSeparator();
    transNode->addChild(trans);
    transNode->addChild(sepChild);

    sep->addChild(transNode);
    return sep;
}

void urdf2inventor::addVisual(SoSeparator * addToNode, SoNode * visual, const Eigen::Vector3d& pos,
    const Eigen::Quaterniond& rot, float _marker_size, SoMaterial * mat)
{
    SoTransform * trans = new SoTransform();
    trans->translation.setValue(pos.x(), pos.y(), pos.z());
    trans->rotation.setValue(rot.x(), rot.y(), rot.z(), rot.w());
    SoSeparator * transSep = new SoSeparator();
    transSep->addChild(trans);
    transSep->addChild(visual);
    if (mat) addToNode->addChild(mat);
    addToNode->addChild(transSep);
}

void urdf2inventor::addSphere(SoSeparator * addToNode, const Eigen::Vector3d& pos, float radius,
    float r, float g, float b)
{
    SoSphere * s = new SoSphere();
    s->radius = radius;
    SoMaterial * mat = new SoMaterial();
    mat->diffuseColor.setValue(r,g,b);
    mat->ambientColor.setValue(0.2, 0.2, 0.2);
    mat->transparency.setValue(0);
    Eigen::Quaterniond rot;
    rot.setIdentity();
    addVisual(addToNode, s, pos, rot, radius, mat);
}

void urdf2inventor::addCylinder(SoSeparator * addToNode, const Eigen::Vector3d& pos,
    const Eigen::Quaterniond rot,
    float radius, float height,
    float r, float g, float b)
{
    SoCylinder * c = new SoCylinder();
    c->radius = radius;
    c->height = height;


    // SoCylinder is oriented along y axis, so change this to z axis
    // and also translate such that it extends along +z
    Eigen::Quaterniond toZ = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0,1,0), Eigen::Vector3d(0,0,1));
    typedef Eigen::Transform<double, 3, Eigen::Affine> EigenTransform;
    SoTransform * trans = new SoTransform();
    trans->translation.setValue(0,0,height/2);
    trans->rotation.setValue(toZ.x(), toZ.y(), toZ.z(), toZ.w());
    SoSeparator * cylinder = new SoSeparator();
    cylinder->addChild(trans);
    cylinder->addChild(c);

    SoMaterial * mat = new SoMaterial();
    mat->diffuseColor.setValue(r,g,b);
    mat->ambientColor.setValue(0.2, 0.2, 0.2);
    mat->transparency.setValue(0);
/*    // SoCylinder is oriented along y axis, so change this to z axis
    // and also translate such that it extends along +z
    Eigen::Quaterniond toZ = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0,1,0), Eigen::Vector3d(0,0,1));
    typedef Eigen::Transform<double, 3, Eigen::Affine> EigenTransform;
    EigenTransform trans;
    trans.setIdentity();
    trans.translate(Eigen::Vector3d(0,0,height/2.0));
    Eigen::Vector3d position=trans*pos;
    Eigen::Quaterniond rotation = rot * toZ;*/
    addVisual(addToNode, cylinder, pos, rot, radius, mat);
}

void urdf2inventor::addLocalAxes(SoSeparator * addToNode, float axesRadius, float axesLength)
{
    float rx, gx, bx, ry, gy, by, rz, gz, bz;
    rx = ry = rz = gx = gy = gz = bx = by = bz = 0;
    rx = 1;  // x axis red
    gy = 1;  // y axis green
    bz = 1;  // z axis blue

    Eigen::Quaterniond rot;
    rot.setIdentity();
    // z axis
    addCylinder(addToNode,Eigen::Vector3d(0,0,0), rot, axesRadius, axesLength ,rz,gz,bz);
    
    Eigen::Vector3d x(1,0,0);
    Eigen::Vector3d y(0,1,0);
    Eigen::Vector3d z(0,0,1);

    // y axis
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z, y);
    addCylinder(addToNode,Eigen::Vector3d(0,0,0), q, axesRadius, axesLength, ry, gy, by);
    
    // x axis
    q = Eigen::Quaterniond::FromTwoVectors(z, x);
    addCylinder(addToNode,Eigen::Vector3d(0,0,0), q, axesRadius, axesLength, rx, gx, bx);
}
