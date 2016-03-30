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

#include <urdf2inventor/Helpers.h>
#include <urdf2inventor/Urdf2Inventor.h>

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <ivcon/ivconv.h>

#include <Inventor/SoDB.h>      // for file reading
#include <Inventor/SoInput.h>   // for file reading
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/nodekits/SoNodeKit.h>

#include <map>
#include <vector>
#include <set>
#include <fstream>

#define RAD_TO_DEG 180/M_PI

// Filename for temporary output file
#define TEMP_STDOUT "/tmp/redirectedStdOut"

#define REDIRECT_STDOUT

// directory where to dump temporary mesh
// files. Only required to read collada files.
#define TEMP_MESH_DIR "/tmp/tempMeshes"

using urdf2inventor::Urdf2Inventor;

std::string Urdf2Inventor::OUTPUT_EXTENSION = ".iv";
std::string Urdf2Inventor::MESH_OUTPUT_DIRECTORY_NAME = "iv";


std::string Urdf2Inventor::getRootLinkName() const
{
    LinkConstPtr root = this->robot.getRoot();
    if (!root.get())
    {
        ROS_ERROR("Loaded model has no root");
        return "";
    }
    return root->name;
}

bool Urdf2Inventor::scaleTranslation(JointPtr& joint, double scale_factor)
{
    EigenTransform vTrans = getTransform(joint);
    scaleTranslation(vTrans, scale_factor);
    setTransform(vTrans, joint);
}

void Urdf2Inventor::scaleTranslation(LinkPtr& link, double scale_factor)
{
    for (std::vector<VisualPtr >::iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        VisualPtr visual = *vit;
        EigenTransform vTrans = getTransform(visual->origin);
        scaleTranslation(vTrans, scale_factor);
        setTransform(vTrans, visual->origin);
    }


    for (std::vector<CollisionPtr >::iterator cit = link->collision_array.begin();
            cit != link->collision_array.end(); ++cit)
    {
        CollisionPtr coll = *cit;
        EigenTransform vTrans = getTransform(coll->origin);
        scaleTranslation(vTrans, scale_factor);
        setTransform(vTrans, coll->origin);
    }
    if (!link->inertial.get())
    {
        // ROS_WARN("Link %s  has no inertial",link->name.c_str());
        return;
    }

    EigenTransform vTrans = getTransform(link->inertial->origin);
    scaleTranslation(vTrans, scale_factor);
    setTransform(vTrans, link->inertial->origin);
}


bool Urdf2Inventor::scale()
{
    if (fabs(scaleFactor-1.0)<1e-04)
    {
        //ROS_INFO("Scale factor 1, so no need to scale model");
        return true;
    }
    ROS_INFO("############### Scaling model");

    if (!scaleModelRecursive(scaleFactor))
    {
        ROS_ERROR("Could not scale up the model");
        return false;
    }
    isScaled = true;

    return true;
}


int Urdf2Inventor::scaleModel(RecursionParamsPtr& p)
{
    FactorRecursionParams::Ptr param = architecture_binding_ns::dynamic_pointer_cast<FactorRecursionParams>(p);
    if (!param.get())
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    LinkPtr link = param->link;
    if (!link.get())
    {
        ROS_ERROR("Recursion parameter must have initialised link!");
        return -1;
    }
    scaleTranslation(link, param->factor);
    JointPtr pjoint = link->parent_joint;
    if (pjoint.get())
    {
        scaleTranslation(pjoint, param->factor);
    }
    return 1;
}

bool Urdf2Inventor::scaleModelRecursive(double scale_factor)
{
    LinkPtr root_link = this->robot.root_link_;
    if (!root_link.get())
    {
        ROS_ERROR("No root link");
        return false;
    }

    // do one call of scaleModel(RecursionParams) for the root link
    LinkPtr parent;  // leave parent NULL
    RecursionParamsPtr p(new FactorRecursionParams(parent, root_link, 0, scale_factor));
    int cvt = scaleModel(p);

    if (cvt < 0)
    {
        ROS_ERROR("Could not convert root mesh");
        return false;
    }

    // go through entire tree
    return (cvt == 0) ||
           (this->traverseTreeTopDown(root_link, boost::bind(&Urdf2Inventor::scaleModel, this, _1), p) >= 0);
}

int Urdf2Inventor::convertMesh(RecursionParamsPtr& p)
{
    // ROS_INFO("convert mesh for %s",link->name.c_str());

    MeshConvertRecursionParams::Ptr param = architecture_binding_ns::dynamic_pointer_cast<MeshConvertRecursionParams>(p);
    if (!param.get())
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    LinkPtr link = param->link;

    SoNode * allVisuals = getAllVisuals(link, param->factor);

    if (!allVisuals)
    {
        ROS_ERROR("Could not get visuals");
        return -1;
    }

    std::string resultFileContent;
    if (!writeInventorFileString(allVisuals, resultFileContent))
    {
        ROS_ERROR("Could not get the mesh file content");
        return -1;
    }

    // ROS_INFO_STREAM("Result file content: "<<resultFileContent);

    if (!param->resultMeshes.insert(std::make_pair(link->name, resultFileContent)).second)
    {
        ROS_ERROR("Could not insert the resulting mesh file for link %s to the map", link->name.c_str());
        return -1;
    }
    return 1;
}

bool Urdf2Inventor::convertMeshes(const std::string& fromLinkName,
                                 const std::string& material,
                                 std::map<std::string, MeshFormat>& meshes)
{
    LinkPtr from_link;
    this->robot.getLink(fromLinkName, from_link);
    if (!from_link.get())
    {
        ROS_ERROR("Link %s does not exist", fromLinkName.c_str());
        return false;
    }

    // do one call of convertMeshes
    LinkPtr parent;  // leave parent NULL
    MeshConvertRecursionParams::Ptr meshParams(new MeshConvertRecursionParams(parent, from_link, 0,
            scaleFactor, material));

    RecursionParamsPtr p(meshParams);
    int cvt = convertMesh(p);
    if (cvt < 0)
    {
        ROS_ERROR("Could not convert root mesh");
        return false;
    }

    // go through entire tree
    int ret = (cvt == 0) ||
              (this->traverseTreeTopDown(from_link, boost::bind(&Urdf2Inventor::convertMesh, this, _1), p) >= 0);

    meshes = meshParams->resultMeshes;
    return ret;
}

Urdf2Inventor::ConversionResultPtr Urdf2Inventor::convert(const std::string& rootLink,
        const std::string& material)
{
    ConversionResultPtr res(new ConversionResultT(OUTPUT_EXTENSION, MESH_OUTPUT_DIRECTORY_NAME));
    res->success = false;
    if (!isScaled && !scale())
    {
        ROS_ERROR("Failed to scale model");
    }

    ROS_INFO("############### Converting meshes");

    if (!convertMeshes(rootLink, material, res->meshes))
    {
        ROS_ERROR("Could not convert meshes");
        return res;
    }

    res->success = true;
    return res;
}

bool Urdf2Inventor::printModel(const std::string& fromLink)
{
    // get root link
    LinkPtr root_link;
    this->robot.getLink(fromLink, root_link);
    if (!root_link)
    {
        ROS_ERROR("no root link %s", this->robot.getName().c_str());
        return false;
    }

    ROS_INFO("Root link: %s", root_link->name.c_str());

    // go through entire tree
    RecursionParamsPtr p(new RecursionParams());
    return this->traverseTreeTopDown(root_link, boost::bind(&Urdf2Inventor::printLink, this, _1), p) >= 0;
}

bool Urdf2Inventor::printModel()
{
    // get root link
    LinkPtr root_link = this->robot.root_link_;
    if (!root_link)
    {
        ROS_ERROR("no root link %s", this->robot.getName().c_str());
        return false;
    }

    ROS_INFO("Root link: %s", root_link->name.c_str());

    // go through entire tree
    RecursionParamsPtr p(new RecursionParams());
    return this->traverseTreeTopDown(root_link, boost::bind(&Urdf2Inventor::printLink, this, _1), p) >= 0;
}


int Urdf2Inventor::printLink(RecursionParamsPtr& p)
{
    LinkPtr parent = p->parent;
    LinkPtr link = p->link;
    unsigned int level = p->level;

    std::stringstream _indent;
    for (unsigned int i = 0; i < level; ++i) _indent << "   ";
    std::string indent = _indent.str();

    std::string pjoint;
    if (link->parent_joint.get()) pjoint = link->parent_joint->name;
    ROS_INFO("%sInformation about %s: parent joint %s", indent.c_str(), link->name.c_str(), pjoint.c_str());

    // get translation
    double x = link->parent_joint->parent_to_joint_origin_transform.position.x;
    double y = link->parent_joint->parent_to_joint_origin_transform.position.y;
    double z = link->parent_joint->parent_to_joint_origin_transform.position.z;
    ROS_INFO("%s Translation: %f %f %f (%f %f %f)",
             indent.c_str(), x, y, z, x * scaleFactor, y * scaleFactor, z * scaleFactor);

    double qx = link->parent_joint->parent_to_joint_origin_transform.rotation.x;
    double qy = link->parent_joint->parent_to_joint_origin_transform.rotation.y;
    double qz = link->parent_joint->parent_to_joint_origin_transform.rotation.z;
    double qw = link->parent_joint->parent_to_joint_origin_transform.rotation.w;
    ROS_INFO("%s Quaternion: %f %f %f %f", indent.c_str(), qx, qy, qz, qw);

    // get rpy
    double roll, pitch, yaw;
    link->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);

    if (isnan(roll) || isnan(pitch) || isnan(yaw))
    {
        ROS_ERROR("getRPY() returned nan!");
        return -1;
    }

    ROS_INFO("%s  (=RPY: %f %f %f)", indent.c_str(), roll, pitch, yaw);
    return 1;
}


bool Urdf2Inventor::getJointNames(const std::string& fromLink, const bool skipFixed, std::vector<std::string>& result)
{
    std::string rootLink=fromLink;
    if (rootLink.empty()){
        rootLink = getRootLinkName();
    }

    // get root link
    LinkPtr root_link;
    this->robot.getLink(rootLink, root_link);
    if (!root_link)
    {
        ROS_ERROR("no root link %s", this->robot.getName().c_str());
        return false;
    }

    ROS_INFO("Get joint names starting from link: %s", root_link->name.c_str());

    // go through entire tree
    StringVectorRecursionParams * stringParams=new StringVectorRecursionParams(skipFixed);
    RecursionParamsPtr p(stringParams);
    bool success = (this->traverseTreeTopDown(root_link, boost::bind(&Urdf2Inventor::getJointNames, this, _1), p) >= 0);
    if (success)
    {
        result=stringParams->names;
    }
    return success;
}


int Urdf2Inventor::getJointNames(RecursionParamsPtr& p)
{
    StringVectorRecursionParams::Ptr param = architecture_binding_ns::dynamic_pointer_cast<StringVectorRecursionParams>(p);
    if (!param.get())
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    LinkPtr parent = p->parent;
    LinkPtr link = p->link;
    unsigned int level = p->level;

    // only return parent joints starting from first recursion level,
    // because we want only the joints in-between the starting linke
    // and the end of the chain.
    //if (level <=1 ) return 1;

    std::string pjoint;
    if (link->parent_joint.get())
    {
        pjoint = link->parent_joint->name;
        if (!param->skipFixed || isActive(link->parent_joint))
            param->names.push_back(pjoint);
    }
    //ROS_INFO("Information about %s: parent joint %s (recursion level %i)", link->name.c_str(), pjoint.c_str(), level);
    return 1;
}




int Urdf2Inventor::traverseTreeTopDown(const LinkPtr& link, boost::function< int(RecursionParamsPtr&)> link_cb,
                                      RecursionParamsPtr& params, unsigned int level)
{
    level += 1;
    for (std::vector<LinkPtr>::const_iterator child = link->child_links.begin();
            child != link->child_links.end(); child++)
    {
        LinkPtr childLink = *child;
        if (childLink.get())
        {
            params->setParams(link, childLink, level);
            int link_ret = link_cb(params);
            if (link_ret <= 0)
            {
                // stopping traversal
                return link_ret;
            }

            // recurse down the tree
            int ret = traverseTreeTopDown(childLink, link_cb, params, level);
            if (ret < 0)
            {
                ROS_ERROR("Error parsing branch of %s", childLink->name.c_str());
                return -1;
            }
        }
        else
        {
            ROS_ERROR("root link: %s has a null child!", link->name.c_str());
            return false;
        }
    }
    return 1;
};

bool Urdf2Inventor::traverseTreeBottomUp(LinkPtr& link, boost::function< LinkPtr(LinkPtr&)> link_cb)
{
    std::set<std::string> toTraverse;
    for (unsigned int i = 0; i < link->child_links.size(); ++i)
    {
        LinkPtr childLink = link->child_links[i];
        toTraverse.insert(childLink->name);
    }

    for (std::set<std::string>::iterator it = toTraverse.begin(); it != toTraverse.end(); ++it)
    {
        LinkPtr childLink;
        this->robot.getLink(*it, childLink);

        if (childLink.get())
        {
            // ROS_INFO("Traversal into child %s",childLink->name.c_str());
            // recurse down the tree
            if (!traverseTreeBottomUp(childLink, link_cb))
            {
                ROS_ERROR("Error parsing branch of %s", childLink->name.c_str());
                return false;
            }
        }
        else
        {
            ROS_ERROR("root link: %s has a null child!", link->name.c_str());
            return false;
        }
    }

    // ROS_INFO("Relink child %s",link->name.c_str());
    link = link_cb(link);
    if (!link.get())
    {
        ROS_ERROR("Error parsing branch of %s", link->name.c_str());
        return false;
    }
    return true;
}


int Urdf2Inventor::checkActiveJoints(RecursionParamsPtr& p)
{
    LinkPtr parent = p->parent;
    LinkPtr link = p->link;
    unsigned int level = p->level;
    if (link->parent_joint.get() && !isActive(link->parent_joint))
    {
        ROS_ERROR("C: Found fixed link %s", link->parent_joint->name.c_str());
        return -1;
    }
    return 1;
}

bool Urdf2Inventor::hasFixedJoints(LinkPtr& from_link)
{
    LinkPtr empty;
    LinkPtr l = from_link;
    RecursionParamsPtr p(new RecursionParams(empty, l, 0));
    if ((checkActiveJoints(p) < 0) ||
            (this->traverseTreeTopDown(from_link,
             boost::bind(&Urdf2Inventor::checkActiveJoints, this, _1), p) < 0))
    {
        return true;
    }
    return false;
}

bool Urdf2Inventor::joinFixedLinks(const std::string& from_link)
{
    LinkPtr link;
    this->robot.getLink(from_link, link);
    return joinFixedLinks(link);
}

bool Urdf2Inventor::joinFixedLinks(LinkPtr& from_link)
{
    if (!this->traverseTreeBottomUp(from_link, boost::bind(&Urdf2Inventor::joinFixedLinksOnThis, this, _1)))
    {
        ROS_ERROR("Could not join fixed links");
        return false;
    }

    // consistency check: All joints in the tree must be active now!
    if (hasFixedJoints(from_link))
    {
        ROS_ERROR("consistency: We should now only have active joitns in the tree!");
        return false;
    }
    return true;
}

Urdf2Inventor::LinkPtr Urdf2Inventor::joinFixedLinksOnThis(LinkPtr& link)
{
    if (!link.get()) return link;

    // ROS_INFO("Traverse %s",link->name.c_str());


    JointPtr jointToParent = link->parent_joint;
    if (!jointToParent.get())
    {
        // ROS_WARN("End of chain at %s, because of no parent joint", link->name.c_str());
        return link;
    }


    LinkPtr parentLink;
    this->robot.getLink(jointToParent->parent_link_name, parentLink);
    if (!parentLink.get())
    {
        ROS_WARN("End of chain at %s, because of no parent link", link->name.c_str());
        return link;
    }

    /*if (link->child_joints.empty()) {
        ROS_WARN("INFO: end effector %s",link->name.c_str());
        return link;
    }*/


    if (isActive(jointToParent))
    {
        // ROS_INFO("Parent of %s is active so won't delete",link->name.c_str());
        // We won't delete this joint, as it is active.
        // ROS_INFO("Joining chain finished between %s and %s",parentLink->name.c_str(),link->name.c_str());
        return link;
    }

    // this joint is fixed, so we will delete it

    // ROS_INFO("Joining between %s and %s",parentLink->name.c_str(),link->name.c_str());
    // remove this link from the parent
    for (std::vector<LinkPtr >::iterator pc = parentLink->child_links.begin();
            pc != parentLink->child_links.end(); pc++)
    {
        LinkPtr child = (*pc);
        if (child->name == link->name)
        {
            // ROS_WARN("Remove link %s",link->name.c_str());
            parentLink->child_links.erase(pc);
            break;
        }
    }
    for (std::vector<JointPtr >::iterator pj = parentLink->child_joints.begin();
            pj != parentLink->child_joints.end(); pj++)
    {
        JointPtr child = (*pj);
        // this child joint is the current one that we just now removed
        if (child->name == jointToParent->name)
        {
            // ROS_WARN("Remove joint %s",child->name.c_str());
            parentLink->child_joints.erase(pj);
            break;
        }
    }

    // the local transfrom of the parent joint
    EigenTransform localTrans = getTransform(link);

    // all this link's child joints now must receive the extra transform from this joint which we removed.
    // then, the joints should be added to the parent link
    for (std::vector<JointPtr >::iterator j = link->child_joints.begin(); j != link->child_joints.end(); j++)
    {
        JointPtr child = (*j);
        if (!isActive(child))
        {
            ROS_ERROR("consistency: At this stage, we should only have active joints, found joint %s!",
                      child->name.c_str());
        }
        EigenTransform vTrans = getTransform(child);

        vTrans = localTrans * vTrans;
        setTransform(vTrans, child);
        child->parent_link_name = parentLink->name;
        parentLink->child_joints.push_back(child);

        // this link's child link has to be added to parents as well
        LinkPtr childChildLink;
        this->robot.getLink(child->child_link_name, childChildLink);
        if (!childChildLink.get())
        {
            ROS_ERROR("consistency: found null child link for joint %s", child->name.c_str());
        }
        parentLink->child_links.push_back(childChildLink);
        childChildLink->setParent(parentLink);
    }

    for (std::vector<VisualPtr >::iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        VisualPtr visual = *vit;
        // apply the transform to the visual before adding it to the parent
        EigenTransform vTrans = getTransform(visual->origin);
        vTrans = localTrans * vTrans;
        setTransform(vTrans, visual->origin);
        parentLink->visual_array.push_back(visual);
    }


    for (std::vector<CollisionPtr >::iterator cit = link->collision_array.begin();
            cit != link->collision_array.end(); ++cit)
    {
        CollisionPtr coll = *cit;
        // apply the transform to the visual before adding it to the parent
        EigenTransform vTrans = getTransform(coll->origin);
        vTrans = localTrans * vTrans;
        setTransform(vTrans, coll->origin);
        parentLink->collision_array.push_back(coll);
    }


    if (parentLink->visual.get()) parentLink->visual.reset();
    if (parentLink->collision.get()) parentLink->collision.reset();


    // combine inertials
    // parent->inertial=XXX TODO;

    return parentLink;
}


std::vector<Urdf2Inventor::JointPtr> Urdf2Inventor::getChain(const LinkPtr& from_link, const LinkPtr& to_link) const
{
    std::vector<JointPtr> chain;

    if (to_link->name == from_link->name) return chain;

    LinkPtr curr = to_link;
    LinkPtr pl = to_link->getParent();

    while (curr.get() && (curr->name != from_link->name))
    {
        JointPtr pj = curr->parent_joint;
        if (!pj.get())
        {
            ROS_ERROR("Null parent joint found! %s", curr->name.c_str());
            return chain;
        }
        chain.push_back(pj);
        curr = pl;
        pl = curr->getParent();
        // ROS_INFO("Parent of %s %s",curr->name.c_str(),pl->name.c_str());
    }
    if (curr->name != from_link->name)
    {
        ROS_ERROR("Failed to find parent chain!");
        return std::vector<JointPtr>();
    }

    std::reverse(chain.begin(), chain.end());

    return chain;
}


std::string Urdf2Inventor::getStdOutRedirectFile()
{
    /*std::stringstream str;
    str << outputDir.c_str() << "/" << TEMP_STDOUT;
    return str.str();*/
    return TEMP_STDOUT;
}

SoNode * Urdf2Inventor::convertMeshFile(const std::string& filename, double scale_factor)
{
    std::string fileExt = urdf2inventor::helpers::fileExtension(filename.c_str());
//    ROS_INFO_STREAM("Filename extension: "<<fileExt);
    SoNode * result=NULL;
    if (fileExt == "dae")
    {
        ROS_INFO_STREAM("Need to convert mesh file "<<filename<<" to temporary stl first");
        ROS_ERROR("This still needs to be implemented! See a possibility in package assimp_mesh_converter");
        // could write temporary meshes to TEMP_MESH_DIR
        // after conversion, we need to 
    }
    else
    {
        // ROS_INFO_STREAM("Converting mesh "<<filename);
        result = convertMeshFileIvcon(filename,scale_factor);
        // ROS_INFO("Converted.");
    } 
    return result;
}

SoNode * Urdf2Inventor::convertMeshFileIvcon(const std::string& filename, double scale_factor)
{
    // int ssize=10000;
    // char bigOutBuf[ssize];
#ifdef REDIRECT_STDOUT
    urdf2inventor::helpers::redirectStdOut(getStdOutRedirectFile().c_str());
#endif
    // first, convert file to inventor. ivconv writes to file only so we have to
    // temporarily write it to file and then read it again
    IVCONV::SCALE_FACTOR = scale_factor;
    IVCONV ivconv;
    ROS_INFO_STREAM("IVCONV reading "<<filename);
    if (!ivconv.read(filename))
    {
        ROS_ERROR("Can't read mesh file %s", filename.c_str());
        return NULL;
    }
    ROS_INFO_STREAM("IVCONV writing "<<filename);

    if (!ivconv.write(TMP_FILE_IV))
    {
        ROS_ERROR("Can't write mesh file %s", TMP_FILE_IV);
        return NULL;
    }
    ROS_INFO_STREAM("IVCONV finished reading "<<filename);

    SoInput in;
    SoNode  *scene = NULL;
    if (!in.openFile(TMP_FILE_IV)) return NULL;
    SoDB::read(&in, scene);

    in.closeFile();

#ifdef REDIRECT_STDOUT
    urdf2inventor::helpers::resetStdOut();
    // ROS_INFO("We got %s",bigOutBuf);
#endif
    return scene;
}

void Urdf2Inventor::cleanup(bool deleteOutputRedirect)
{
    const char * stdOutRedirectFile = getStdOutRedirectFile().c_str();
    if (deleteOutputRedirect &&
            urdf2inventor::helpers::fileExists(stdOutRedirectFile))
    {
        urdf2inventor::helpers::deleteFile(stdOutRedirectFile);
    }
    if (urdf2inventor::helpers::fileExists(TMP_FILE_IV))
    {
        urdf2inventor::helpers::deleteFile(TMP_FILE_IV);
    }
}

bool Urdf2Inventor::writeInventorFile(SoNode * node, const std::string& filename)
{
    SoOutput out;
    if (!out.openFile(filename.c_str())) return false;
    out.setBinary(false);
    SoWriteAction write(&out);
    write.apply(node);
    write.getOutput()->closeFile();
    return true;
}

bool Urdf2Inventor::writeInventorFileString(SoNode * node, std::string& result)
{
    SoOutput out;
    out.setBinary(false);
    size_t initBufSize = 100;
    void * buffer = malloc(initBufSize * sizeof(char));
    out.setBuffer(buffer, initBufSize, std::realloc);
    SoWriteAction write(&out);
    write.apply(node);

    void * resBuf = NULL;
    size_t resBufSize = 0;

    if (!out.getBuffer(resBuf, resBufSize) || (resBufSize == 0))
    {
        ROS_ERROR("Failed to write file string to buffer.");
        return false;
    }

    result = std::string(static_cast<char*>(resBuf), resBufSize);  // buffer will be copied

    free(resBuf);

    return true;
}

SoSeparator * Urdf2Inventor::addSubNode(SoNode * addAsChild, SoNode* parent, SoTransform * trans)
{
    SoSeparator * sep = dynamic_cast<SoSeparator*>(parent);
    if (!sep)
    {
        ROS_ERROR("parent is not a separator");
        return NULL;
    }

    SoSeparator * sepChild = dynamic_cast<SoSeparator*>(addAsChild);
    if (!sepChild)
    {
        ROS_ERROR("child is not a separator");
        return NULL;
    }

    // ROS_WARN_STREAM("######### Adding transform "<<trans->translation<<", "<<trans->rotation);

    SoSeparator * transNode = new SoSeparator();
    transNode->addChild(trans);
    transNode->addChild(sepChild);

    sep->addChild(transNode);
    return sep;
}

SoSeparator * Urdf2Inventor::addSubNode(SoNode * addAsChild, SoNode* parent, EigenTransform& eTrans)
{
    SoTransform * transform = getTransform(eTrans);
    return addSubNode(addAsChild, parent, transform);
}


SoNode * Urdf2Inventor::getAllVisuals(const LinkPtr link, double scale_factor, bool scaleUrdfTransforms)
{
    SoNodeKit::init();
    SoNode * allVisuals = new SoSeparator();
    allVisuals->ref();
    std::string linkName = link->name;
    unsigned int i = 0;
    for (std::vector<VisualPtr >::const_iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        VisualPtr visual = (*vit);
        GeometryPtr geom = visual->geometry;

        EigenTransform vTransform = getTransform(visual->origin);
        if (scaleUrdfTransforms) scaleTranslation(vTransform, scale_factor);

        if (geom->type == urdf::Geometry::MESH)
        {
            MeshPtr mesh = architecture_binding_ns::dynamic_pointer_cast<urdf::Mesh>(geom);
            if (!mesh.get())
            {
                ROS_ERROR("Mesh cast error");
                return NULL;
            }
            std::string meshFilename = urdf2inventor::helpers::packagePathToAbsolute(mesh->filename);

            SoNode * somesh = convertMeshFile(meshFilename, scale_factor);
            if (!somesh)
            {
                ROS_ERROR("Mesh could not be read");
                return NULL;
            }
            std::stringstream str;
            str << "_visual_" << i << "_" << linkName;
            somesh->setName(str.str().c_str());
            allVisuals = addSubNode(somesh, allVisuals, vTransform);
        }
        else
        {
            ROS_ERROR("Only support mesh files so far");
            return NULL;
        }
        ++i;
    }

    std::stringstream str;
    str << "_" << linkName;

    allVisuals->setName(str.str().c_str());

    return allVisuals;
}

SoTransform * Urdf2Inventor::getTransform(const EigenTransform& eTrans)
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

Urdf2Inventor::LinkPtr Urdf2Inventor::getLink(const std::string& name)
{
    LinkPtr ptr;
    this->robot.getLink(name, ptr);
    return ptr;
}


Urdf2Inventor::JointPtr Urdf2Inventor::getJoint(const std::string& name)
{
    JointPtr ptr;
    if (this->robot.joints_.find(name) == this->robot.joints_.end()) ptr.reset();
    else ptr = this->robot.joints_.find(name)->second;
    return ptr;
}

void Urdf2Inventor::setTransform(const EigenTransform& t, urdf::Pose& p)
{
    Eigen::Vector3d trans(t.translation());
    Eigen::Quaterniond rot(t.rotation());

    p.position.x = trans.x();
    p.position.y = trans.y();
    p.position.z = trans.z();
    p.rotation.x = rot.x();
    p.rotation.y = rot.y();
    p.rotation.z = rot.z();
    p.rotation.w = rot.w();
}

void Urdf2Inventor::setTransform(const EigenTransform& t, JointPtr& joint)
{
    setTransform(t, joint->parent_to_joint_origin_transform);
}

void Urdf2Inventor::scaleTranslation(EigenTransform& t, double scale_factor)
{
    Eigen::Vector3d trans = t.translation();
    trans *= scale_factor;
    Eigen::Matrix3d rot = t.rotation();
    t.setIdentity();
    t.translate(trans);
    t.rotate(rot);
}

Urdf2Inventor::EigenTransform Urdf2Inventor::getTransform(const urdf::Pose& p)
{
    urdf::Vector3 _jtr = p.position;
    Eigen::Vector3d jtr(_jtr.x, _jtr.y, _jtr.z);
    urdf::Rotation _jrot = p.rotation;
    Eigen::Quaterniond jrot(_jrot.w, _jrot.x, _jrot.y, _jrot.z);
    jrot.normalize();
    EigenTransform tr;
    tr.setIdentity();
    tr = tr.translate(jtr);
    tr = tr.rotate(jrot);
    return tr;
}

Eigen::Matrix4d Urdf2Inventor::getTransformMatrix(const LinkPtr& from_link,  const LinkPtr& to_link)
{
    if (from_link->name == to_link->name) return Eigen::Matrix4d::Identity();

    std::vector<JointPtr> pjoints = getChain(from_link, to_link);

    if (pjoints.empty())
    {
        ROS_ERROR("could not get chain from %s to %s", from_link->name.c_str(), to_link->name.c_str());
        return Eigen::Matrix4d::Identity();
    }

    // ROS_INFO("Chain from %s to %s",from_link->name.c_str(),to_link->name.c_str());

    Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();

    for (std::vector<JointPtr>::iterator it = pjoints.begin(); it != pjoints.end(); ++it)
    {
        // ROS_INFO("Chain joint %s",(*it)->name.c_str());
        Eigen::Matrix4d mat = getTransform(*it).matrix();
        ret *= mat;
    }
    return ret;
}

Urdf2Inventor::EigenTransform Urdf2Inventor::getTransform(const LinkPtr& from_link,  const JointPtr& to_joint)
{
    LinkPtr link1 = from_link;
    LinkPtr link2;
    this->robot.getLink(to_joint->child_link_name, link2);
    if (!link1.get() || !link2.get())
    {
        ROS_ERROR("Invalid joint specifications (%s, %s), first needs parent and second child",
                  link1->name.c_str(), link2->name.c_str());
    }
    return getTransform(link1, link2);
}
    

SoNode * Urdf2Inventor::loadAndGetAsInventor(const std::string& urdfFilename, const std::string from_link, bool useScaleFactor)
{
    if (!loadModelFromFile(urdfFilename))
    {
        ROS_ERROR("Could not load file");
        return NULL;
    }

    ROS_INFO("Converting files for robot %s", getRobotName().c_str());
    std::string rootLink=from_link;
    if (rootLink.empty()){
        rootLink = getRootLinkName();
    }


    std::vector<std::string> jointNames;
    if (!getJointNames(rootLink, true, jointNames))
    {
        ROS_WARN("Could not retrieve joint names to print on screen");
    }
    else
    {
        ROS_INFO_STREAM("Joint names starting from "<<rootLink<<":");
        for (int i=0; i<jointNames.size(); ++i) ROS_INFO_STREAM(jointNames[i]);
    }
    return getAsInventor(rootLink, useScaleFactor);
}


SoNode * Urdf2Inventor::getAsInventor(const LinkPtr& from_link, bool useScaleFactor)
{
    SoNode * allVisuals = getAllVisuals(from_link, useScaleFactor ? scaleFactor : 1.0, useScaleFactor);
    if (!allVisuals)
    {
        ROS_ERROR("Could not get visuals");
        return NULL;
    }
    for (std::vector<JointPtr>::const_iterator pj = from_link->child_joints.begin();
            pj != from_link->child_joints.end(); pj++)
    {
        LinkPtr childLink;
        this->robot.getLink((*pj)->child_link_name, childLink);
        SoNode * childNode = getAsInventor(childLink, useScaleFactor);
        if (!childNode)
        {
            ROS_ERROR_STREAM("Could not get child node for "<<childLink->name);
            return NULL;
        }
        EigenTransform jointTransform = getTransform(*pj);
        if (useScaleFactor) scaleTranslation(jointTransform, scaleFactor);

        // ROS_WARN_STREAM("Transform joint "<<(*pj)->name<<": "<<jointTransform);
        //ROS_INFO_STREAM("Adding sub node for "<<childLink->name);

        allVisuals = addSubNode(childNode, allVisuals, jointTransform);
    }

    return allVisuals;
}


SoNode * Urdf2Inventor::getAsInventor(const std::string& fromLink, bool useScaleFactor)
{
    std::string rootLink=fromLink;
    if (rootLink.empty()){
        rootLink = getRootLinkName();
    }
//    ROS_INFO_STREAM("Getting as inventor starting from '"<<rootLink<<"'");
    LinkPtr from_link;
    this->robot.getLink(rootLink, from_link);
    if (!from_link.get())
    {
        ROS_ERROR_STREAM("Cannot find link '"<<rootLink<<"'");
        return NULL;
    }
    return getAsInventor(from_link, useScaleFactor);
}


bool Urdf2Inventor::writeAsInventor(const std::string& ivFilename, const std::string& fromLink, bool useScaleFactor)
{
    std::string rootLink=fromLink;
    if (rootLink.empty()){
        rootLink = getRootLinkName();
    }
    LinkPtr from_link;
    this->robot.getLink(rootLink, from_link);
    if (!from_link.get())
    {
        ROS_ERROR_STREAM("Cannot find link '"<<rootLink<<"'");
        return false;
    }
    ROS_INFO_STREAM("Writing from link '"<<rootLink<<"' to file "<<ivFilename);
    return writeAsInventor(ivFilename, from_link, useScaleFactor);
}

bool Urdf2Inventor::writeAsInventor(const std::string& ivFilename, const LinkPtr& from_link, bool useScaleFactor)
{
    SoNode * inv = getAsInventor(from_link, useScaleFactor);
    if (!inv)
    {
        ROS_ERROR("could not generate overall inventor file");
        return false;
    }
    return writeInventorFile(inv, ivFilename);
}


bool Urdf2Inventor::loadModelFromFile(const std::string& urdfFilename)
{
    std::string xml_file;
    if (!getModelFromFile(urdfFilename, xml_file))
    {
        ROS_ERROR("Could not load file");
        return false;
    }

    if (!loadModelFromXMLString(xml_file))
    {
        ROS_ERROR("Could not load file");
        return false;
    }
    return true;
}



bool Urdf2Inventor::loadModelFromXMLString(const std::string& xmlString)
{
    bool success = robot.initString(xmlString);
    if (!success)
    {
        ROS_ERROR("Could not load model from XML string");
        return false;
    }
    isScaled = false;
    return true;
}

bool Urdf2Inventor::loadModelFromParameterServer()
{
    if (!robot.initParam("robot_description")) return false;
    isScaled = false;
    return true;
}


bool Urdf2Inventor::getModelFromFile(const std::string& filename, std::string& xml_string) const
{
    std::fstream xml_file(filename.c_str(), std::fstream::in);
    if (xml_file.is_open())
    {
        while (xml_file.good())
        {
            std::string line;
            std::getline(xml_file, line);
            xml_string += (line + "\n");
        }
        xml_file.close();
        return true;
    }
    else
    {
        ROS_ERROR("Could not open file [%s] for parsing.", filename.c_str());
    }
    return false;
}


Urdf2Inventor::ConversionResultPtr Urdf2Inventor::loadAndConvert(const std::string& urdfFilename,
        const std::string& convertFromLink,
        const std::string& material)
{
    ConversionResultPtr result(new ConversionResultT(OUTPUT_EXTENSION, MESH_OUTPUT_DIRECTORY_NAME));
    result->success = false;
    
    if (!loadModelFromFile(urdfFilename))
    {
        ROS_ERROR("Could not load file");
        return result;
    }

    ROS_INFO("Converting files for robot %s", getRobotName().c_str());
    std::string rootLink = convertFromLink; 
    if (rootLink.empty())
    {
        rootLink = getRootLinkName();
    }
    

    if (!joinFixedLinks(rootLink))
    {
        ROS_ERROR("Could not traverse");
        return result;
    }
    // ROS_INFO("00000000000000000000000");
    // p.printModel(rootLink);

    result = convert(rootLink, material);
    if (!result->success)
    {
        ROS_ERROR("Could not do the conversion");
        return result;
    }
    
    // ROS_INFO_STREAM("Contacts generated: "<<result.contacts);
    return result;
}

