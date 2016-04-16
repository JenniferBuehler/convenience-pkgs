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
#include <urdf2inventor/IVHelpers.h>

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <ivcon/ivconv.h>

#include <Inventor/SoDB.h>      // for file reading
#include <Inventor/SoInput.h>   // for file reading
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/nodekits/SoNodeKit.h>

#include <map>
#include <vector>
#include <set>
#include <fstream>
#include <algorithm>

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

    MeshConvertRecursionParams::Ptr param = architecture_binding_ns::dynamic_pointer_cast<MeshConvertRecursionParams>(p);
    if (!param.get())
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    LinkPtr link = param->link;
    
    ROS_INFO("Convert mesh for link '%s'",link->name.c_str());

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
    std::string rootLink=fromLinkName;
    if (rootLink.empty()){
        rootLink = getRootLinkName();
    }
    
    LinkPtr from_link;
    this->robot.getLink(rootLink, from_link);
    if (!from_link.get())
    {
        ROS_ERROR("Link %s does not exist", rootLink.c_str());
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

Urdf2Inventor::ConversionResultPtr Urdf2Inventor::preConvert(const ConversionParametersPtr& params)
{
    ConversionResultPtr res(new ConversionResultT(OUTPUT_EXTENSION, MESH_OUTPUT_DIRECTORY_NAME));
    res->success=false;
    return res;
}

Urdf2Inventor::ConversionResultPtr Urdf2Inventor::postConvert(const ConversionParametersPtr& params, ConversionResultPtr& result)
{
    result->success = true;
    return result;
}


Urdf2Inventor::ConversionResultPtr Urdf2Inventor::convert(const ConversionParametersPtr& params)
{
    ConversionResultPtr res = preConvert(params);
    if (!res.get())
    {
        ROS_ERROR("Failed to pre-convert");
        return res;
    }
    res->success = false;

    if (!isScaled && !scale())
    {
        ROS_ERROR("Failed to scale model");
    }

    ROS_INFO("############### Converting meshes");

    if (!convertMeshes(params->rootLinkName, params->material, res->meshes))
    {
        ROS_ERROR("Could not convert meshes");
        return res;
    }

    return postConvert(params, res);
}



int Urdf2Inventor::addJointLink(RecursionParamsPtr& p)
{
    OrderedJointsRecursionParams::Ptr param = architecture_binding_ns::dynamic_pointer_cast<OrderedJointsRecursionParams>(p);
    if (!param.get())
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    // ROS_INFO("At link %s", parent->name.c_str());
    if (param->parent->child_joints.empty())
    {
        ROS_ERROR("If links are connected, there must be at least one joint");
        return -1;
    }
    if (!param->link->parent_joint.get())
    {
        ROS_ERROR("NULL parent joint");
        return -1;
    }
    if (param->parent->child_joints.size() > 1)
    {
        if (!param->allowSplits)
        {
            ROS_ERROR("Splitting point at %s!", param->parent->name.c_str());
            return -1;
        }
        // this is a splitting point, we have to add support for this
    }

    if (param->onlyActive && !isActive(param->link->parent_joint))
    {
        // ROS_INFO("No type");
        return 1;
    }


    // ROS_INFO("Adding %s",link->parent_joint->name.c_str());
    param->dependencyOrderedJoints.push_back(param->link->parent_joint);
    return 1;
}


bool Urdf2Inventor::getDependencyOrderedJoints(std::vector<JointPtr>& result,
        const JointPtr& from_joint, bool allowSplits, bool onlyActive)
{
    LinkPtr childLink;
    getRobot().getLink(from_joint->child_link_name, childLink);
    if (!childLink.get())
    {
        ROS_ERROR("Child link %s not found", from_joint->child_link_name.c_str());
        return false;
    }
    if (!getDependencyOrderedJoints(result, childLink, allowSplits, onlyActive))
    {
        ROS_ERROR("Could not get ordered joints for %s", from_joint->child_link_name.c_str());
        return false;
    }
    if (!onlyActive || isActive(from_joint))
    {
        result.insert(result.begin(), from_joint);
    }
    return true;
}

bool Urdf2Inventor::getDependencyOrderedJoints(std::vector<JointPtr>& result, const LinkPtr& from_link,
        bool allowSplits, bool onlyActive)
{
    if (!allowSplits && (from_link->child_joints.size() > 1))
    {
        ROS_ERROR("Splitting point at %s!", from_link->name.c_str());
        return false;
    }
    OrderedJointsRecursionParams * p = new OrderedJointsRecursionParams(allowSplits, onlyActive);
    RecursionParamsPtr rp(p);
    if (this->traverseTreeTopDown(from_link, boost::bind(&Urdf2Inventor::addJointLink, this, _1), rp) < 0)
    {
        ROS_ERROR("Could not add depenency order");
        p->dependencyOrderedJoints.clear();
        return false;
    }

    result = p->dependencyOrderedJoints;
    return true;
}



int Urdf2Inventor::getChildJoint(const JointPtr& joint, JointPtr& child)
{
    LinkPtr childLink = getChildLink(joint);
    if (!childLink.get())
    {
        ROS_ERROR("Consistency: all joints must have child links");
        return -2;
    }
    if (childLink->child_joints.size() > 1)
    {
        return -1;
    }
    if (childLink->child_joints.empty())
    {   // this is the end link, and we've defined the end
        // frame to be at the same location as the last joint,
        // so no rotation should be needed?
        return 0;
    }
    // there must be only one joint
    child = childLink->child_joints.front();
    return 1;
}

Urdf2Inventor::LinkPtr Urdf2Inventor::getChildLink(const JointPtr& joint)
{
    LinkPtr childLink;
    getRobot().getLink(joint->child_link_name, childLink);
    return childLink;
}

Urdf2Inventor::LinkConstPtr Urdf2Inventor::readChildLink(const JointPtr& joint) const
{
    LinkPtr childLink;
    getRobot().getLink(joint->child_link_name, childLink);
    return childLink;
}



Urdf2Inventor::JointPtr Urdf2Inventor::getParentJoint(const JointPtr& joint)
{
    LinkConstPtr parentLink = getRobot().getLink(joint->parent_link_name);
    if (!parentLink.get()) return JointPtr();
    return parentLink->parent_joint;
}

Urdf2Inventor::JointConstPtr Urdf2Inventor::readParentJoint(const JointPtr& joint) const
{
    LinkConstPtr parentLink = getRobot().getLink(joint->parent_link_name);
    if (!parentLink.get()) return JointPtr();
    return parentLink->parent_joint;
}





bool Urdf2Inventor::isChildOf(const LinkConstPtr& parent, const LinkConstPtr& child) const
{
    for (unsigned int i = 0; i < parent->child_links.size(); ++i)
    {
        LinkPtr childLink = parent->child_links[i];
        if (childLink->name == child->name) return true;
    }
    return false;
}


bool Urdf2Inventor::isChildJointOf(const LinkConstPtr& parent, const JointConstPtr& joint) const
{
    for (unsigned int i = 0; i < parent->child_joints.size(); ++i)
    {
        JointPtr childJnt = parent->child_joints[i];
        if (childJnt->name == joint->name) return true;
    }
    return false;
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


bool Urdf2Inventor::hasChildLink(const LinkConstPtr& link, const std::string& childName) const
{
    for (unsigned int i = 0; i < link->child_links.size(); ++i)
    {
        LinkPtr childLink = link->child_links[i];
        if (childLink->name == childName) return true;
    }
    return false;
}


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
        if (!hasChildLink(link,*it))
        {
            ROS_ERROR_STREAM("Consistency: Link "<<link->name<<" does not have child "<<*it<<" any more.");
            return false;
        }
        
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
    if (level==0) return 1;
    if (link->parent_joint.get() && !isActive(link->parent_joint))
    {
        ROS_INFO("Urdf2Inventor: Found fixed joint %s", link->parent_joint->name.c_str());
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
    // ROS_INFO_STREAM("### Joining fixed links starting from "<<fromLinkName);
    LinkPtr link;
    this->robot.getLink(from_link, link);
    if (!link.get())
    {
        ROS_ERROR_STREAM("No link named "<<from_link);
        return false;
    }
    return joinFixedLinks(link);
}

bool Urdf2Inventor::joinFixedLinks(LinkPtr& from_link)
{
    ROS_INFO_STREAM("### Joining fixed links starting from "<<from_link->name
        <<" with "<<from_link->child_links.size()<<" children");

    // call joinFixedLinksOnThis only from child
    // links of from_link, because the joining happens with the
    // parent joint.
    std::set<std::string> toTraverse;
    for (unsigned int i = 0; i < from_link->child_links.size(); ++i)
    {
        LinkPtr childLink = from_link->child_links[i];
        toTraverse.insert(childLink->name);
    }

    for (std::set<std::string>::iterator it = toTraverse.begin(); it != toTraverse.end(); ++it)
    {
        if (!hasChildLink(from_link,*it))
        {
            ROS_WARN_STREAM("Urdf2Inventor, test case: Link "<<from_link->name
                <<" does not have child "<<*it<<" any more, this can happen if it was joined.");
            continue;
        }
        
        LinkPtr childLink;
        this->robot.getLink(*it, childLink);
 
        if (!this->traverseTreeBottomUp(childLink, boost::bind(&Urdf2Inventor::joinFixedLinksOnThis, this, _1)))
        {
            ROS_ERROR_STREAM("Could not join fixed links from link "<<childLink->name);
            return false;
        }
    }
/*
    // OLD: join fixed joints incl. parent joint
    if (!this->traverseTreeBottomUp(from_link, boost::bind(&Urdf2Inventor::joinFixedLinksOnThis, this, _1)))
    {
        ROS_ERROR("Could not join fixed links");
        return false;
    }
*/
    
    // consistency check: All joints in the tree must be active now!
    if (hasFixedJoints(from_link))
    {
        ROS_ERROR("consistency: We should now only have active joints in the tree!");
        return false;
    }
    return true;
}

Urdf2Inventor::LinkPtr Urdf2Inventor::joinFixedLinksOnThis(LinkPtr& link)
{
    if (!link.get()) return link;

    // ROS_INFO("--joinFixedLinksOnThis: Traverse %s",link->name.c_str());
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
        // ROS_INFO("Parent of %s (%s) is active so won't delete",link->name.c_str(), jointToParent->name.c_str());
        // We won't delete this joint, as it is active.
        // ROS_INFO("Joining chain finished between %s and %s",parentLink->name.c_str(),link->name.c_str());
        return link;
    }

    // this joint is fixed, so we will delete it
    // ROS_INFO("Joining fixed joint (%s) between %s and %s",jointToParent->name.c_str(), parentLink->name.c_str(),link->name.c_str());
    
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
    // ROS_INFO_STREAM("Transform between "<<parentLink->name<<" and "<<link->name<<": "<<localTrans);
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
        EigenTransform jTrans = getTransform(child);

        jTrans = localTrans * jTrans;
        setTransform(jTrans, child);
#if 0
        EigenTransform rotAxTrans=jTrans.inverse();
        // ROS_INFO_STREAM("Transforming rotation axis by "<<rotAxTrans);
        // ROS_INFO_STREAM("Old child axis of "<<child->name<<": "<<child->axis);
        applyTransform(rotAxTrans,child->axis); 
        /*Eigen::Vector3d childAxis(child->axis.x, child->axis.y, child->axis.z);
        childAxis.normalize();
        child->axis.x=childAxis.x();
        child->axis.y=childAxis.y();
        child->axis.z=childAxis.z();*/
        // ROS_INFO_STREAM("New child axis of "<<child->name<<": "<<child->axis);
#endif
        // add this child joint to the parent
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
        EigenTransform jTrans = getTransform(visual->origin);
        jTrans = localTrans * jTrans;
        setTransform(jTrans, visual->origin);
        parentLink->visual_array.push_back(visual);
    }


    for (std::vector<CollisionPtr >::iterator cit = link->collision_array.begin();
            cit != link->collision_array.end(); ++cit)
    {
        CollisionPtr coll = *cit;
        // apply the transform to the visual before adding it to the parent
        EigenTransform jTrans = getTransform(coll->origin);
        jTrans = localTrans * jTrans;
        setTransform(jTrans, coll->origin);
        parentLink->collision_array.push_back(coll);
    }


    if (parentLink->visual.get()) parentLink->visual.reset();
    if (parentLink->collision.get()) parentLink->collision.reset();


    // combine inertials
    // parent->inertial=XXX TODO;

    return parentLink;
}


std::vector<Urdf2Inventor::JointPtr> Urdf2Inventor::getChain(const LinkConstPtr& from_link, const LinkConstPtr& to_link) const
{
    std::vector<JointPtr> chain;

    if (to_link->name == from_link->name) return chain;

    LinkConstPtr curr = to_link;
    LinkConstPtr pl = to_link->getParent();

    while (curr.get() && (curr->name != from_link->name))
    {
        // ROS_INFO_STREAM("Chain: "<<curr->name);
        JointPtr pj = curr->parent_joint;
        if (!pj.get())
        {
            ROS_ERROR("Urdf2Inventor: End of chain at link '%s'", curr->name.c_str());
            return std::vector<JointPtr>();
        }
        chain.push_back(pj);
        curr = pl;
        pl = curr->getParent();
        // if (pl.get()) ROS_INFO_STREAM("Parent: "<<pl->name);
    }
    if (curr->name != from_link->name)
    {
        ROS_ERROR_STREAM("Urdf2Inventor: could not find link "<<
            from_link->name<<" while traversing up the chain starting from "<<
            to_link->name<<". Failed to find parent chain!");
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
    std::string fileExtLow=fileExt;
    boost::to_lower(fileExtLow);
    if ((fileExtLow != "stl") && (fileExtLow != "obj"))
    {
        ROS_INFO_STREAM("Need to convert mesh file "<<filename<<" to temporary obj/stl first!");
        ROS_ERROR("This still needs to be implemented! See a possibility in package assimp_mesh_converter");
        // could write temporary meshes to TEMP_MESH_DIR
        // after conversion, we need to 
/*
        SoInput in;
        SoNode  *scene = NULL;
        if (!in.openFile(filename.c_str()))
        {
            ROS_ERROR_STREAM("Could not open file "<<filename);
            return NULL;
        }
        scene = SoDB::readAll(&in);
        if (!scene)
        {
            ROS_ERROR_STREAM("Could not read file "<<filename);
            return NULL;

        }
        in.closeFile();
        return scene;*/
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
    //ROS_INFO_STREAM("IVCONV reading "<<filename);
#ifdef REDIRECT_STDOUT
    urdf2inventor::helpers::redirectStdOut(getStdOutRedirectFile().c_str());
#endif
    // first, convert file to inventor. ivconv writes to file only so we have to
    // temporarily write it to file and then read it again
    IVCONV::SCALE_FACTOR = scale_factor;
    IVCONV ivconv;
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

    SoInput in;
    SoNode  *scene = NULL;
    if (!in.openFile(TMP_FILE_IV)) return NULL;
    SoDB::read(&in, scene);

    in.closeFile();

#ifdef REDIRECT_STDOUT
    urdf2inventor::helpers::resetStdOut();
    // ROS_INFO("We got %s",bigOutBuf);
#endif
    //ROS_INFO_STREAM("IVCONV finished reading "<<filename);
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
            allVisuals = urdf2inventor::addSubNode(somesh, allVisuals, vTransform);
        }
        else
        {
            ROS_ERROR_STREAM("Only support mesh files so far, but have urdf::Geometry type "<<geom->type);
            return NULL;
        }
        ++i;
    }

    std::stringstream str;
    str << "_" << linkName;

    allVisuals->setName(str.str().c_str());

    return allVisuals;
}

/*SoTransform * Urdf2Inventor::getTransform(const EigenTransform& eTrans) const
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
}*/

Urdf2Inventor::LinkPtr Urdf2Inventor::getLink(const std::string& name)
{
    LinkPtr ptr;
    this->robot.getLink(name, ptr);
    return ptr;
}

Urdf2Inventor::LinkConstPtr Urdf2Inventor::readLink(const std::string& name) const
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

Urdf2Inventor::JointConstPtr Urdf2Inventor::readJoint(const std::string& name) const
{
    JointConstPtr ptr;
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

void Urdf2Inventor::applyTransform(const EigenTransform& t, urdf::Vector3& v)
{
    Eigen::Vector3d _v(v.x,v.y,v.z);
    Eigen::Quaterniond rot(t.rotation());
    //ROS_INFO_STREAM("Rotation: "<<rot);
    _v=rot*_v;
    v.x=_v.x();
    v.y=_v.y();
    v.z=_v.z();
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

Urdf2Inventor::EigenTransform Urdf2Inventor::getTransform(const urdf::Pose& p) const
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

Eigen::Matrix4d Urdf2Inventor::getTransformMatrix(const LinkConstPtr& from_link,  const LinkConstPtr& to_link) const
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

Urdf2Inventor::EigenTransform Urdf2Inventor::getTransform(const LinkPtr& from_link,  const JointPtr& to_joint) const
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


bool equalAxes(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2)
{
    Eigen::Vector3d _z1=z1;
    Eigen::Vector3d _z2=z2;
    _z1.normalize();
    _z2.normalize();
    double dot = _z1.dot(_z2);
    return (std::fabs(dot - 1.0)) < U2G_EPSILON;
    // float alpha = acos(z1.dot(z2));
    // return (std::fabs(alpha) < U2G_EPSILON);
}


bool Urdf2Inventor::jointTransformForAxis(const urdf::Joint& joint,
        const Eigen::Vector3d& axis, Eigen::Quaterniond& rotation)
{
    Eigen::Vector3d rotAxis(joint.axis.x, joint.axis.y, joint.axis.z);
    rotAxis.normalize();
    // ROS_INFO_STREAM("Rotation axis for joint "<<joint.name<<": "<<rotAxis);
    if (equalAxes(rotAxis, axis)) return false;

    rotation = Eigen::Quaterniond::FromTwoVectors(rotAxis, axis);
    // ROS_WARN_STREAM("z alignment: "<<rotation);
    return true;
}

bool Urdf2Inventor::allRotationsToAxis(const std::string& fromLinkName, const Eigen::Vector3d& axis)
{
    ROS_INFO_STREAM("### Transforming all rotations starting from "<<fromLinkName<<" to axis "<<axis);
    LinkPtr from_link;
    getRobot().getLink(fromLinkName, from_link);
    if (!from_link.get())
    {
        ROS_ERROR("Link %s does not exist", fromLinkName.c_str());
        return false;
    }

    for (std::vector<JointPtr>::iterator pj = from_link->child_joints.begin();
            pj != from_link->child_joints.end(); pj++)
    {
        if (!allRotationsToAxis(*pj, axis))
        {
            ROS_ERROR("Aborting recursion.");
            return false;
        }
    }
    return true;
}


bool Urdf2Inventor::allRotationsToAxis(JointPtr& joint, const Eigen::Vector3d& axis)
{
    LinkPtr childLink;
    getRobot().getLink(joint->child_link_name, childLink);

    if (!childLink.get())
    {
        ROS_ERROR("All joints must have a child link!");
        return false;
    }
    Eigen::Quaterniond alignAxis;
    if (jointTransformForAxis(*joint, axis, alignAxis))
    {
        ROS_INFO_STREAM("Transforming axis for joint "<<joint->name<<" with transform "<<alignAxis);
        applyTransform(joint, EigenTransform(alignAxis), false);
        // the link has to receive the inverse transorm, so it stays at the original position
        Eigen::Quaterniond alignAxisInv = alignAxis.inverse();
        applyTransform(childLink, EigenTransform(alignAxisInv), true);

        // now, we have to fix the child joint's (1st order child joints) transform
        // to correct for this transformation.
        for (std::vector<JointPtr>::iterator pj = childLink->child_joints.begin();
                pj != childLink->child_joints.end(); pj++)
        {
            applyTransform(*pj, EigenTransform(alignAxisInv), true);
        }

        // finally, set the rotation axis to the target
        joint->axis.x = axis.x();
        joint->axis.y = axis.y();
        joint->axis.z = axis.z();
    }
    // recurse
    for (std::vector<JointPtr>::iterator pj = childLink->child_joints.begin();
            pj != childLink->child_joints.end(); pj++)
    {
        if (!allRotationsToAxis(*pj, axis))
        {
            ROS_ERROR("Aborting recursion.");
            return false;
        }
    }
    return true;
}


bool Urdf2Inventor::applyTransform(JointPtr& joint, const EigenTransform& trans, bool preMult)
{
    EigenTransform vTrans = getTransform(joint);
    if (preMult) vTrans = trans * vTrans;
    else vTrans = vTrans * trans;
    setTransform(vTrans, joint);
}

void Urdf2Inventor::applyTransform(LinkPtr& link, const EigenTransform& trans, bool preMult)
{
    // ROS_INFO("applying transform to link %s",link->name.c_str());

    for (std::vector<VisualPtr >::iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        VisualPtr visual = *vit;
        EigenTransform vTrans = getTransform(visual->origin);
        // ROS_INFO_STREAM("a visual for link"<<link->name<<" with transform "<<vTrans);
        if (preMult) vTrans = trans * vTrans;
        else vTrans = vTrans * trans;
        setTransform(vTrans, visual->origin);
    }


    for (std::vector<CollisionPtr >::iterator cit = link->collision_array.begin();
            cit != link->collision_array.end(); ++cit)
    {
        CollisionPtr coll = *cit;
        EigenTransform vTrans = getTransform(coll->origin);
        if (preMult) vTrans = trans * vTrans;
        else vTrans = vTrans * trans;
        setTransform(vTrans, coll->origin);
    }

    EigenTransform vTrans = getTransform(link->inertial->origin);
    if (preMult) vTrans = trans * vTrans;
    else vTrans = vTrans * trans;
    setTransform(vTrans, link->inertial->origin);
}


void Urdf2Inventor::printJointNames(const std::string& fromLink) 
{
    std::vector<std::string> jointNames;
    if (!getJointNames(fromLink, false, jointNames))
    {
        ROS_WARN("Could not retrieve joint names to print on screen");
    }
    else
    {
        ROS_INFO_STREAM("Joint names starting from "<<fromLink<<":");
        for (int i=0; i<jointNames.size(); ++i) ROS_INFO_STREAM(jointNames[i]);
    }
}
    

SoNode * Urdf2Inventor::loadAndGetAsInventor(const std::string& urdfFilename, const std::string from_link, bool useScaleFactor)
{
    if (!loadModelFromFile(urdfFilename))
    {
        ROS_ERROR("Could not load file");
        return NULL;
    }

    ROS_INFO("Converting robot %s URDF to inventor", getRobotName().c_str());
    std::string rootLink=from_link;
    if (rootLink.empty()){
        rootLink = getRootLinkName();
    }
    printJointNames(from_link);
    std::vector<std::string> jointNames;
    if (!getJointNames(rootLink, false, jointNames))
    {
        ROS_WARN("Could not retrieve joint names to print on screen");
    }
    else
    {
        ROS_INFO_STREAM("Joint names starting from "<<rootLink<<":");
        for (int i=0; i<jointNames.size(); ++i) ROS_INFO_STREAM(jointNames[i]);
    }
    return getAsInventor(rootLink, useScaleFactor, addAxes, axesRadius, axesLength);
}


void Urdf2Inventor::addLocalAxes(const LinkConstPtr& link, SoSeparator * addToNode, bool useScaleFactor,
    float _axesRadius, float _axesLength) const
{
    bool active=isActive(link->parent_joint);
    float rad = _axesRadius;
    float rotRad = _axesRadius/3;
    float h=_axesLength;
    float rotH=h*1.8;
    float rtr, rtg, rtb;
    // rotation axis pink
    rtr = rtb = 1;
    rtg = 0;
    if (useScaleFactor)
    {
        rad*=scaleFactor;
        h*=scaleFactor;
        rotRad*=scaleFactor;
        rotH*=scaleFactor;
    }
    if (!active)
    {
        rad/=2;
        h*=1.5;
    }
    urdf2inventor::addLocalAxes(addToNode, rad, h);
   
    Eigen::Vector3d rotAxis=getRotationAxis(link->parent_joint);
    Eigen::Vector3d z(0,0,1);
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z, rotAxis);
    urdf2inventor::addCylinder(addToNode,Eigen::Vector3d(0,0,0), q, rotRad,rotH,rtr,rtg,rtb);
}


SoNode * Urdf2Inventor::getAsInventor(const LinkPtr& from_link, bool useScaleFactor,
    bool _addAxes, float _axesRadius, float _axesLength)
{
    // ROS_INFO_STREAM("Get all visuals of "<<from_link->name<<" (parent joint "<<from_link->parent_joint->name<<")");
    SoNode * allVisuals = getAllVisuals(from_link, useScaleFactor ? scaleFactor : 1.0, useScaleFactor);
    if (!allVisuals)
    {
        ROS_ERROR("Could not get visuals");
        return NULL;
    }

    if (_addAxes) 
    {
        SoSeparator * _allVisuals = dynamic_cast<SoSeparator*>(allVisuals);
        if (!_allVisuals)
        {
            ROS_WARN_STREAM("The node for link "<<from_link->name<<" is not a separator, so cannot add axes");
        }
        else 
        {
            addLocalAxes(from_link, _allVisuals, useScaleFactor, _axesRadius, _axesLength);
        }
    }

    for (std::vector<JointPtr>::const_iterator pj = from_link->child_joints.begin();
            pj != from_link->child_joints.end(); pj++)
    {
        LinkPtr childLink;
        this->robot.getLink((*pj)->child_link_name, childLink);
        SoNode * childNode = getAsInventor(childLink, useScaleFactor, _addAxes, _axesRadius, _axesLength);
        if (!childNode)
        {
            ROS_ERROR_STREAM("Could not get child node for "<<childLink->name);
            return NULL;
        }
        EigenTransform jointTransform = getTransform(*pj);
        if (useScaleFactor) scaleTranslation(jointTransform, scaleFactor);

        // ROS_WARN_STREAM("Transform joint "<<(*pj)->name<<": "<<jointTransform);
        //ROS_INFO_STREAM("Adding sub node for "<<childLink->name);

        allVisuals = urdf2inventor::addSubNode(childNode, allVisuals, jointTransform);
    }

    return allVisuals;
}


SoNode * Urdf2Inventor::getAsInventor(const std::string& fromLink, bool useScaleFactor,
    bool _addAxes, float _axesRadius, float _axesLength)
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
    return getAsInventor(from_link, useScaleFactor, _addAxes, _axesRadius, _axesLength);
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
    SoNode * inv = getAsInventor(from_link, useScaleFactor, addAxes, axesRadius, axesLength);
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
    robot_urdf=xmlString;
    return true;
}

/*bool Urdf2Inventor::loadModelFromParameterServer()
{
    if (!robot.initParam("robot_description")) return false;
    isScaled = false;
    return true;
}*/


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
        bool joinFixed,
        const ConversionParametersPtr& params)
{
    ConversionResultPtr failResult(new ConversionResultT(OUTPUT_EXTENSION, MESH_OUTPUT_DIRECTORY_NAME));
    failResult->success = false;

    ROS_INFO_STREAM("Loading model from file "<<urdfFilename);

    if (!loadModelFromFile(urdfFilename))
    {
        ROS_ERROR("Could not load file");
        return failResult;
    }


    ROS_INFO("Converting files for robot %s", getRobotName().c_str());
    std::string rootLink = params->rootLinkName; 
    if (rootLink.empty())
    {
        rootLink = getRootLinkName();
    }

    LinkPtr _rootLink = getLink(rootLink);
    if (!_rootLink.get())
    {
        ROS_ERROR("Root link %s does not exist",rootLink.c_str());
        return failResult;
    }

    if (joinFixed) ROS_INFO("Joining fixed links..");
    if (joinFixed && !joinFixedLinks(rootLink))
    {
        ROS_ERROR("Could not join fixed links");
        return failResult;
    }

    // ROS_INFO("00000000000000000000000");
    // p.printModel(rootLink);

    ConversionResultPtr result = convert(params);
    if (!result.get() || !result->success)
    {
        ROS_ERROR("Could not do the conversion");
        return result;
    }
    
    // ROS_INFO_STREAM("Contacts generated: "<<result.contacts);
    return result;
}

