# urdf2inventor

To display a URDF file:

``roslaunch urdf2inventor urdf_viewer.launch urdf_file:=<your-file>``

The URDF file to view or convert has to be URDF, **not xacro**.

To convert a URDF file:

``roslaunch urdf2inventor urdf2inventor.launch urdf_file:=<your-file> output_dir:=<your-output-directory>``
