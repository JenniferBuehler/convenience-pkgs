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

#ifndef URDF2INVENTOR_URDF2INVENTOR_H
#define URDF2INVENTOR_URDF2INVENTOR_H
// Copyright Jennifer Buehler

//-----------------------------------------------------
#include <urdf/model.h>

#include <iostream>
#include <string>
#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Inventor/nodes/SoTransform.h>

#include <urdf2inventor/ConversionResult.h>
#include <architecture_binding/SharedPtr.h>

// epsilon value to use when comparing whether two axes are the same
// (difference of angle between axes)
#define U2G_EPSILON 1e-07

// this is a temporary filename (extension .iv) which is needed for internal usage, but can be deleted after execution.
#define TMP_FILE_IV "/tmp/urdf2inventor_tmp.iv"

namespace urdf2inventor
{

/**
 * \brief This class provides functions to transform a robot described in URDF to the inventor format.
 *
 * Careful: So far, only meshes with .stl and .obj extensions have been tested. There were problems
 * with some .stl meshes and ivcon however (it idles forever, crashes, or even freezes the screen).
 * So far, the best solution is to convert all meshes to .obj beforehand. Package assimp_mesh_converter
 * can be used to do this. At some time (hopefully soon) this will be automated here, and dependency to
 * package ivcon should be removed.
 * 
 * TODO: This should be separated in 2 different hierarchies, one to handle the URDF traversal
 * to apply all sorts of functions, and another to do the operations such as mesh conversion and model scaling.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class Urdf2Inventor
{
protected:
    typedef Eigen::Transform<double, 3, Eigen::Affine> EigenTransform;

public:
    typedef architecture_binding::shared_ptr<const urdf::Joint>::type JointConstPtr;
    typedef architecture_binding::shared_ptr<const urdf::Link>::type LinkConstPtr;
    typedef architecture_binding::shared_ptr<urdf::Joint>::type JointPtr;
    typedef architecture_binding::shared_ptr<urdf::Link>::type LinkPtr;

    // the .iv files are represented as strings
    typedef std::string MeshFormat;
    typedef ConversionResult<MeshFormat> ConversionResultT;
    typedef architecture_binding::shared_ptr<ConversionResultT>::type ConversionResultPtr;
    
    typedef architecture_binding::shared_ptr<ConversionParameters>::type ConversionParametersPtr;

    // output file format to convert the meshes to
    static std::string OUTPUT_EXTENSION;

    // within the output directory specified in the node, another directory is going to be created
    // to contain the mesh files. The name of this directory can be specified here.
    static std::string MESH_OUTPUT_DIRECTORY_NAME;

    /**
     * \param _scaleFactor the graspit model might have to be scaled (the urdf model is in meters, graspit! in millimeters).
     * This can be specified with this scale factor.
     */
    explicit Urdf2Inventor(float _scaleFactor = 1):
        scaleFactor(_scaleFactor),
        isScaled(false) {}

    ~Urdf2Inventor()
    {
    }

    /**
     * Reads the URDF file from the filename into \e xml_string. Does not change
     * anything in the Urdf2Inventor object.
     */
    bool getModelFromFile(const std::string& urdfFilename, std::string& xml_string) const;
   
    /**
     * Loads the URDF model from file
     */ 
    bool loadModelFromFile(const std::string& urdfFilename);

    /**
     * Loads the URDF from a file
     */
    bool loadModelFromXMLString(const std::string& xmlString);

    /**
     * Loads the URDF from parameter server
     * TODO: still need to re-activate this by also reading the
     * robot urdf into field \e robot_urdf
     */
    //bool loadModelFromParameterServer(); 

    /**
     * Removes all fixed links in the model by adding visuals and collision geometry to the first parent link which is
     * attached to a non-fixed link. Model has to be loaded with any of the load() methods first. 
     */
    bool joinFixedLinks(const std::string& from_link);

    /**
     * Transform the URDF such that all rotation axises (in the joint's local reference frame) are this axis
     */
    bool allRotationsToAxis(const std::string& fromLinkName, const Eigen::Vector3d& axis);


    /**
     * Returns an inventor node for all links down from (and including) from_link.
     * IMPORTANT: This will also load the model, so any other URDF model previously loaded will be overwritten.
     *
     * \param useScaleFactor if set to true, the model is scaled up using scale factor set in constructor.
     * \param fromLink if empty string, the root link in the URDF is going to be used. Otherwise, a link
     *      name can be set here which will return the model starting from this link name.
     */
    SoNode * loadAndGetAsInventor(const std::string& filename, const std::string fromLink="", bool useScaleFactor = true);

    /**
     * Returns an inventor node for all links down from (and including) \e from_link. The model needs to be loaded
     * first by any of the load functions.
     *
     * \param useScaleFactor if set to true, the model is scaled up using scale factor set in constructor.
     * \param fromLink if empty string, the root link in the URDF is going to be used. Otherwise, a link
     *      name can be set here which will return the model starting from this link name.
     */
    SoNode * getAsInventor(const std::string& fromLink="", bool useScaleFactor=true);

    /**
     * writes all elements down from \e fromLink to files in inventor format.
     * \param outputFilename has to be an inventor filename
     * \param fromLink if empty string, the root link in the URDF is going to be used as starting point. Otherwise, a link
     *      name can be set here which will write the model starting from this link name.
     */
    bool writeAsInventor(const std::string& outputFilename,  const std::string& fromLink = "", bool useScaleFactor = true);

    /**
     * Convenience method: loads the URDF file, then joins fixed links (only if \e joinFixed) and then converts the URDF file to
     * inventor mesh files by calling convert(). 
     * IMPORTANT: This will also load the model, so any other URDF model previously loaded will be overwritten.
     */
    ConversionResultPtr loadAndConvert(const std::string& urdfFilename,
        bool joinFixed,
        const ConversionParametersPtr& params);
    /**
     * \param rootLink if empty string, the root link in the URDF is going to be used. Otherwise, a link
     *      name can be set here which will convert the model starting from this link name.
     */
    ConversionParametersPtr getBasicConversionParams(const std::string& rootLink="", const std::string& material="plastic")
    {
        return ConversionParametersPtr(new ConversionParameters(rootLink,material));
    }

    /**
     * Method which does the conversion from URDF to Inventor. This
     * fist calls the implementation-specific preConvert(),
     * then scales the model (using the scale factor specified
     * in the constructor, unless the model was already scaled before), then converts the individual links mesh files
     * by calling convertMeshes(), and finally calls implementation-specific postConvert().
     * To save an inventor file of the *whole* robot, use writeAsInventor() instead.
     *
     * \param params parameters of the conversion
     */
    virtual ConversionResultPtr convert(const ConversionParametersPtr& params);

    /**
     * Prints the structure of the URDF to standard out
     */
    bool printModel();

    /**
     * prints the URDF model to standard out, starting from this link.
     */
    bool printModel(const std::string& fromLink);

    /**
     * Prints the joint names. Can't be const in this version because
     * of the use of a recursive method.
     */
    void printJointNames(const std::string& fromLink);

    /**
     * Returns all joint names in depth-frist search order starting from \e fromLink (or from root if
     * \e fromLink is empty)
     */
    bool getJointNames(const std::string& fromLink, const bool skipFixed, std::vector<std::string>& result);


    /**
     * Cleans up all temporary files written to disk.
     */
    void cleanup(bool deleteOutputRedirect = true);

    std::string getRootLinkName() const;
   
    std::string getRobotName() const
    { 
        return this->robot.getName();
    }

    std::string getRobotURDF() const
    {
        return robot_urdf;
    }
protected:
    typedef architecture_binding::shared_ptr<urdf::Visual>::type VisualPtr;
    typedef architecture_binding::shared_ptr<urdf::Geometry>::type GeometryPtr;
    typedef architecture_binding::shared_ptr<urdf::Mesh>::type MeshPtr;
    typedef architecture_binding::shared_ptr<urdf::Collision>::type CollisionPtr;

    /**
     * \brief Encapsulates data carried within a recursion (traverseTree() function). At each recursion, parent, link and level fields
     * are set from within traverseTree.
     */
    class RecursionParams
    {
    public:
        typedef architecture_binding::shared_ptr<RecursionParams>::type Ptr;

        RecursionParams(): level(-1) {}
        RecursionParams(LinkPtr& _parent, LinkPtr& _link, unsigned int _level):
            parent(_parent),
            link(_link),
            level(_level) {}
        RecursionParams(const RecursionParams& o):
            parent(o.parent),
            link(o.link),
            level(o.level) {}
        virtual ~RecursionParams() {}

        RecursionParams& operator=(const RecursionParams& o)
        {
            parent = o.parent;
            link = o.link;
            level = o.level;
            return *this;
        }

        // Sets the current variables for the recursion state.
        void setParams(const LinkPtr& _parent, const LinkPtr& _link, int _level)
        {
            parent = _parent;
            link = _link;
            level = _level;
        }

        LinkPtr parent;
        LinkPtr link;
        unsigned int level;
    };

    typedef RecursionParams::Ptr RecursionParamsPtr;

    /**
     * \brief Includes a factor value to be passed on in recursion.
     */
    class FactorRecursionParams: public Urdf2Inventor::RecursionParams
    {
    public:
        typedef architecture_binding::shared_ptr<FactorRecursionParams>::type Ptr;
        FactorRecursionParams(): RecursionParams(), factor(1.0) {}
        FactorRecursionParams(Urdf2Inventor::LinkPtr& _parent,
                              Urdf2Inventor::LinkPtr& _link, int _level, double _factor):
            RecursionParams(_parent, _link, _level),
            factor(_factor) {}
        explicit FactorRecursionParams(double _factor):
            RecursionParams(),
            factor(_factor) {}
        FactorRecursionParams(const FactorRecursionParams& o):
            RecursionParams(o),
            factor(o.factor) {}
        virtual ~FactorRecursionParams() {}

        double factor;
    };


    /**
     * \brief Collects string values into a vector
     */
    class StringVectorRecursionParams: public Urdf2Inventor::RecursionParams
    {
    public:
        typedef architecture_binding::shared_ptr<StringVectorRecursionParams>::type Ptr;
        StringVectorRecursionParams(const bool _skipFixed):
            RecursionParams(),
            skipFixed(_skipFixed) {}
/*        StringVectorRecursionParams(Urdf2Inventor::LinkPtr& _parent,
                              Urdf2Inventor::LinkPtr& _link, int _level,
                              std::vector<std::string> _names, const bool _skipFixed):
            RecursionParams(_parent, _link, _level),
            names(_names),
            skipFixed(_skipFixed) {}*/
        StringVectorRecursionParams(const StringVectorRecursionParams& o):
            RecursionParams(o),
            names(o.names),
            skipFixed(o.skipFixed) {}
        virtual ~StringVectorRecursionParams() {}
        // skip the fixed joints and collect only movable ones
        bool skipFixed;
        std::vector<std::string> names;
    };


     /**
     * \brief Includes parameters to be passed on in recursion when generating meshes.
     */
    class MeshConvertRecursionParams: public FactorRecursionParams
    {
    public:
        typedef architecture_binding::shared_ptr<MeshConvertRecursionParams>::type Ptr;
        MeshConvertRecursionParams(): FactorRecursionParams() {}

        /**
         * \param material the material to use in the converted mesh
         */
        MeshConvertRecursionParams(Urdf2Inventor::LinkPtr& _parent,
                                   Urdf2Inventor::LinkPtr& _link,
                                   int _level,
                                   double _scale_factor,
                                   const std::string& _material):
            FactorRecursionParams(_parent, _link, _level, _scale_factor),
            material(_material) {}

        MeshConvertRecursionParams(double _scale_factor, const std::string _material):
            FactorRecursionParams(_scale_factor),
            material(_material) {}
        MeshConvertRecursionParams(const MeshConvertRecursionParams& o):
            FactorRecursionParams(o),
            material(o.material),
            resultMeshes(o.resultMeshes) {}
        virtual ~MeshConvertRecursionParams() {}

        std::string material;

        // the resulting meshes (inventor files), indexed by the link name
        std::map<std::string, MeshFormat> resultMeshes;
    };
  

    /**
     * \brief Recursion data for getting a list of joints, ordered by dependency (no joint depending on others
     * will come before them in the result vector)
     */
    class OrderedJointsRecursionParams: public Urdf2Inventor::RecursionParams
    {
    public:
        typedef boost::shared_ptr<OrderedJointsRecursionParams> Ptr;
        OrderedJointsRecursionParams(): RecursionParams() {}
        OrderedJointsRecursionParams(bool _allowSplits, bool _onlyActive):
            allowSplits(_allowSplits),
            onlyActive(_onlyActive) {}
        OrderedJointsRecursionParams(const OrderedJointsRecursionParams& o):
            RecursionParams(o) {}
        virtual ~OrderedJointsRecursionParams() {}

        // Result set
        std::vector<Urdf2Inventor::JointPtr> dependencyOrderedJoints;

        // Allow splits, i.e. one link has several child joints. if this is set to false,
        // the recursive operation will fail at splitting points.
        bool allowSplits;

        // Only add joints to the result which are active.
        bool onlyActive;
    };


    // Returns the joint's rotation axis as Eigen Vector
    inline Eigen::Vector3d getRotationAxis(const JointPtr& j) const
    {
        return Eigen::Vector3d(j->axis.x, j->axis.y, j->axis.z);
    }

    // Function for recursive getDependencyOrderedJoints
    int addJointLink(RecursionParamsPtr& p);

    /**
     * Returns all joints starting from from_joint (including from_joint) within the tree. This is obtained by depth-first traversal,
     * so all joints in the result won't depend on any joints further back in the result set.
     */
    bool getDependencyOrderedJoints(std::vector<JointPtr>& result, const JointPtr& from_joint,
                                    bool allowSplits = true, bool onlyActive = true);

    /**
     * Returns all joints down from from_link within the tree. This is obtained by depth-first traversal,
     * so all joints in the result won't depend on any joints further back in the result set.
     */
    bool getDependencyOrderedJoints(std::vector<JointPtr>& result, const LinkPtr& from_link,
                                    bool allowSplits = true, bool onlyActive = true);


    /**
     * \retval -2 on error
     * \retval -1 if the joint has multiple children
     * \retval 0 if the joint has no child joints (it's an end effector joint),
     * \retval 1 if a child joint is returned in parameter "child"
     */
    int getChildJoint(const JointPtr& joint, JointPtr& child);

    bool isChildOf(const LinkConstPtr& parent, const LinkConstPtr& child) const;
    bool isChildJointOf(const LinkConstPtr& parent, const JointConstPtr& joint) const;
    
    LinkPtr getChildLink(const JointPtr& joint);
    LinkConstPtr readChildLink(const JointPtr& joint) const;
    
    JointPtr getParentJoint(const JointPtr& joint);
    JointConstPtr readParentJoint(const JointPtr& joint) const;
 
    /**
     * Method called by convert() which can be implemented by subclasses to return a different type of ConversionResult
     * and do any operations required *before* the main body of convert() is being done.
     * See also postConvert().
     * \param rootLink the conversion is to be done starting from this link.
     * \return NULL/invalid shared ptr if pre-conversion failed.
     */ 
    virtual ConversionResultPtr preConvert(const ConversionParametersPtr& params); 

    /**
     * Method called by convert() which can be implemented by subclasses to do any operations
     * required *after* the main body of convert() has been done.
     * See also preConvert().
     * \param rootLink the conversion is to be done starting from this link.
     * \param result the conversion result so far. This is the object which has been returned by preConvert(),
     *      with changes applied by the main body of convert() applied to the base class fields.
     * \return Has to be the same as \e result, with possible changes applied.
     *      Set ConversionResult::success to false to indicate failure, or to true to indicate success.
     */ 
    virtual ConversionResultPtr postConvert(const ConversionParametersPtr& params, ConversionResultPtr& result); 

    /**
     * Helper function for getJointNames(const std::string&, std::vector<std::string>&).
     */
    int getJointNames(RecursionParamsPtr& p);

    /**
     * Convert all meshes starting from fromLinkName into the inventor format, and store them in the given
     * mesh files container.
     * While converting, the mesh files can be scaled by the scale factor set in the constructor 
     * \param material the material to use in the converted format
     * \param meshes the resulting meshes (inventor files), indexed by the link names
     * \param meshDescXML the resulting GraspIt! XML description files for the meshes, indexed by the link names
     */
    bool convertMeshes(const std::string& fromLinkName, const std::string& material,
                       std::map<std::string, MeshFormat>& meshes);


    /**
     * traverses the tree starting from link, but not including link itself, and calls link_cb on each link.
     * \param link_cb returns -1 or 0 if traversal is to be stopped, otherwise 1. -1 is for stop because error,
     * 0 is stop because an expected condition found
     * \return -1 or 0 if traversal was stopped, otherwise 1. -1 is for stop because error, 0 is stop because an expected
     * condition found in callback function
     */
    int traverseTreeTopDown(const LinkPtr& link, boost::function< int(RecursionParamsPtr&)> link_cb,
                            RecursionParamsPtr& params, unsigned int level = 0);

    /**
     * Similar to traverseTree, but traverses bottom-up and provides ability to re-link tree (by traversing it safely such
     * that changes in structure won't matter)
     */
    bool traverseTreeBottomUp(LinkPtr& link, boost::function< LinkPtr(LinkPtr&)> link_cb);

    /**
     * scale the model and the meshes
     */
    bool scale();
   

    // Returns if this is an active joint in the URDF description
    inline bool isActive(const JointPtr& joint) const
    {
        return (joint->type == urdf::Joint::REVOLUTE) ||
               (joint->type == urdf::Joint::CONTINUOUS) ||
               (joint->type == urdf::Joint::PRISMATIC);
    }

    JointPtr getJoint(const std::string& name);
    JointConstPtr readJoint(const std::string& name) const;
    LinkPtr getLink(const std::string& name);
    LinkConstPtr readLink(const std::string& name) const;

    // Scales up the translation part of the transform t by the given factor
    void scaleTranslation(EigenTransform& t, double scale_factor);

    void setTransform(const EigenTransform& t, urdf::Pose& p);
    void setTransform(const EigenTransform& t, JointPtr& joint);

    // Get joint transform to parent
    EigenTransform getTransform(const urdf::Pose& p) const;

    // Get joint transform to parent
    inline EigenTransform getTransform(const JointConstPtr& joint) const
    {
        return getTransform(joint->parent_to_joint_origin_transform);
    }

    // Get transform to parent link (transform of link's parent joint)
    inline EigenTransform getTransform(const LinkConstPtr& link) const
    {
        return getTransform(link->parent_joint);
    }

    Eigen::Matrix4d getTransformMatrix(const LinkConstPtr& from_link,  const LinkConstPtr& to_link) const;

    inline EigenTransform getTransform(const LinkConstPtr& from_link,  const LinkConstPtr& to_link) const
    {
        return EigenTransform(getTransformMatrix(from_link, to_link));
    }

    EigenTransform getTransform(const LinkPtr& from_link,  const JointPtr& to_joint) const;

    const urdf::Model& getRobot() const
    {
        return robot;
    }
    
    // returns true if there are any fixed joints down from from_link
    bool hasFixedJoints(LinkPtr& from_link);

    inline float getScaleFactor() const
    {
        return scaleFactor;
    }

    /**
     * Applies the transformation on the joint transform
     * \param scaleTransform set to true if the urdf's transforms are to be scaled (using scaleFactor) before applying the transform
     */
    bool applyTransform(JointPtr& joint, const EigenTransform& trans, bool preMult);

    /**
     * Applies the transformation on the link's visuals, collisions and intertial.
     * \param scaleTransform set to true if the urdf's transforms are to be scaled (using scaleFactor) before applying the transform
     */
    void applyTransform(LinkPtr& link, const EigenTransform& trans, bool preMult);

    void applyTransform(const EigenTransform& t, urdf::Vector3& v);

private:

    /**
     * \return true if this joint needs a transformation to align its rotation axis with the given axis.
     * In this case the rotation parameter contains the necessary rotation.
     */
    bool jointTransformForAxis(const urdf::Joint& joint, const Eigen::Vector3d& axis, Eigen::Quaterniond& rotation);

    /**
     * Recursively re-arranges all joint-transforms (starting from joint) and visual/collision/intertial
     * rotations such that all joints rotate around z-axis
     */
    bool allRotationsToAxis(JointPtr& joint, const Eigen::Vector3d& axis);

    /**
     * scales the translation part of the joint transform by the given factor
     */
    bool scaleTranslation(JointPtr& joint, double scale_factor);

    /**
     * scales the translation part of the origins of visuals/collisions/inertial by the given factor
     */
    void scaleTranslation(LinkPtr& link, double scale_factor);

    /**
     * Function to be called during recursion incurred in convertMeshes()
     */
    int convertMesh(RecursionParamsPtr& p);

    /**
     * Function used for recursion by scaleModelRecursive().
     */
    int scaleModel(RecursionParamsPtr& p);

    /**
     * Scales the URDF model by this factor. This means all translation parts of the joint transforms are multiplied by this.
     * The mesh files are not touched, but the visual/collision/intertial translations are scaled as well.
     * Meshes can be scaled using convertMeshes().
     */
    bool scaleModelRecursive(double scale_factor);

    /**
     * printing a link, used by recursive printModel().
     */
    int printLink(RecursionParamsPtr& p);

    /**
     * Function which can be used for recursion which returns -1 if there are any inactive joints in the urdf.
     */
    int checkActiveJoints(RecursionParamsPtr& p);


    /**
     * Helper: If the parent joint of this link is fixed, it will be removed, and this link's visual will be
     * connected to the parent link.
     * If the joint was active, the function returns the same link as in the parameter.
     * Otherwise, it returns the pointer to the parent link which now contains
     * this link's visual/collision.
     */
    LinkPtr joinFixedLinksOnThis(LinkPtr& link);

    /**
     * Removes all fixed links down the chain in the model by adding visuals and collision geometry to the first parent link which is
     * attached to a non-fixed link.
     */
    bool joinFixedLinks(LinkPtr& from_link);

    /**
     * Returns all joints between from_link and to_link
     */
    std::vector<JointPtr> getChain(const LinkConstPtr& from_link, const LinkConstPtr& to_link) const;

    /**
     * Converts a mesh file (given in filename) to an Inventor structure, to which the root is returned.
     * The model may be scaled at the same time using scale_factor.
     */
    SoNode * convertMeshFile(const std::string& filename, double scale_factor);

    /**
     * Uses the ivcon package to conver to inventor file.
     */
    SoNode * convertMeshFileIvcon(const std::string& filename, double scale_factor);

    /**
     * Get the mesh from link, scale it up by scale_factor, and pack it into an SoNode which is also respects the
     * scale_factor in its translations
     * \param scaleUrdfTransforms set to true if the transforms coming from this urdf model should be scaled up as well.
     * If this is false, only the meshes are scaled.
     */
    SoNode * getAllVisuals(const LinkPtr link, double scale_factor, bool scaleUrdfTransforms = false);

    // Returns the transform eTrans as SoTransform node
    SoTransform * getTransform(const EigenTransform& eTrans) const;

    /**
     * Adds a SoNode (addAsChild) as a child (to parent), transformed by the given transform (eTrans)
     * and returns the SoSeparator containing parent with the child.
     */
    SoSeparator * addSubNode(SoNode * addAsChild, SoNode* parent, EigenTransform& eTrans);

    /**
     * Adds a SoNode (addAsChild) as a child (to parent), transformed by the given transform (eTrans) and
     * returns the SoSeparator containing parent with the child.
     */
    SoSeparator * addSubNode(SoNode * addAsChild, SoNode* parent, SoTransform * trans);

    /**
     * Recursive function which returns an inventor node for all links down from (and including) from_link.
     * \param useScaleFactor if set to true, the model is scaled up using scale factor set in constructor.
     */
    SoNode * getAsInventor(const LinkPtr& from_link, bool useScaleFactor);

    /**
     * Writes the contents of SoNode into the file of given name.
     */
    bool writeInventorFile(SoNode * node, const std::string& filename);

    /**
     * writes the contents of SoNode into the inventor (*.iv) format and returns the file
     * content as a string.
     */
    bool writeInventorFileString(SoNode * node, std::string& result);

    /**
     * Writes all elements down from from_link to a file in inventor format.
     * \param outFilename has to be an inventor filename
     */
    bool writeAsInventor(const std::string& outFilename, const LinkPtr& from_link, bool useScaleFactor = true);


    bool hasChildLink(const LinkConstPtr& link, const std::string& childName) const;

    std::string getStdOutRedirectFile();

    urdf::Model robot;
    std::string robot_urdf;

    // The graspit model might ahve to be scaled compared to the urdf model, this is the scale factor which does that.
    float scaleFactor;
    bool isScaled;
};

}  //  namespace urdf2inventor
#endif   // URDF2INVENTOR_URDF2INVENTOR_H
