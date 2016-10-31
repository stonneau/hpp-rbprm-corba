#  Humanoid Path Planner - RBPRM-CORBA module

Copyright 2015, 2016 LAAS-CNRS

Author: Steve Tonneau, Anna Seppala

## Description
hpp-rbprm-corba implements python bindings for hpp-rbprm, and presents a few example files.
Please refer to this [link](https://github.com/stonneau/hpp-rbprm) for information on hpp-rbprm.

## Installation on ubuntu-14.04 64 bit with ros-indigo

To install hpp-rbprm-corba:

  1. Install HPP-RBPRM and its dependencies
	- see https://github.com/stonneau/hpp-rbprm

  2. Install HPP-AFFORDANCE-CORBA along with its dependencies
  - see https://github.com/anna-seppala/hpp-affordance-corba

  3. Use CMake to install the library. For instance:

			mkdir $HPP_RBPRM_CORBA_DIR/build
			cd $HPP_RBPRM_CORBA_DIR/build
			cd cmake ..	
			make install
	


## Documentation

  Open $DEVEL_DIR/install/share/doc/hpp-rbprm-corba/doxygen-html/index.html in a web brower and you
  will have access to the code documentation. If you are using ipython, the documentation of the methods implemented
  is also directly available in a python console.

## Example

  To see the planner in action, two examples from our IJRR submission with HyQ are available. Examples with HRP-2 are also provided,
  though they can only be executed if you have access to HRP-2 model.


  - First of all, retrieve and build the HyQ model from its github repository:
	https://github.com/iit-DLSLab/hyq-description


    ```$ rosrun xacro xacro.py  hyq_description/robots/hyq_model.urdf.xacro -o  hyq.urdf```

  - Make sure to install hyq.urdf in $HPP_DEVEL_DIR/install/share/hpp-rbprm-corba/

  - The planning is decomposed in two phases / scripts. First, a root path is computed (\*_path.py files). Then, the contacts are generated along the computed path (\*_interp.py files). The scripts are located in the folder /scripts/scenarios.

  - To only plan and see the root path, run:


    ```$ ./run.sh darpa_hyq_path.py```

  - To generate the complete contact sequence, run:


    ```$ ./run.sh darpa_hyq_interp.py```

  The scripts include comments explaining the different calls to the library.


## rbprm-plugin for gepetto-gui
## Usage

#### Basic usage of gepetto-gui
Gepetto-gui is a interactive viewer application implemented within the package [gepetto-viewer-corba](https://github.com/anna-seppala/gepetto-viewer-corba/tree/affordance). After installing this package, launch the binary file `gepetto-gui` and do as in [this video](http://homepages.laas.fr/jmirabel/raw/videos/hpp-gui-example.mp4).

#### Adding predefined robots and environments for rbprm-plugin
For convenience, rom robots, full-body robots and environments can be predefined.

The configuration files are - from the installation prefix - in `etc/gepetto-gui`.
Open `${CMAKE_INSTALL_PREFIX}/etc/gepetto-gui/rbprmRoms.conf` and write:
```
[HYQ]
ModelName=hyq_trunk_large
RootJointType=freeflyer
Package=hpp-rbprm-corba
URDFRomNames=hyq_rhleg_rom,hyq_lfleg_rom,hyq_rfleg_rom,hyq_lhleg_rom
PackagePath=/local/anna/devel/install/share/hpp-rbprm-corba
URDFSuffix=
SRDFSuffix=
MeshPackage=hpp-rbprm-corba
MeshDirectory=/local/anna/devel/install/share/
Environment=hpp-rbprm-corba,darpa,planning
SetFilter=hyq_rhleg_rom,hyq_lfleg_rom,hyq_rfleg_rom,hyq_lhleg_rom
AffordanceFilter=Support.Lean,Support,Support,Support.Lean
BoundedJoints=base_joint_xyz
BoundedJointSizes=6
JointBounds=-2,5,-1,1,0.3,4
BoundSO3=-0.4,0.4,-3,3,-3,3
InitialConfig=-2,0,0.63,1,0,0,0
GoalConfig=3,0,0.63,1,0,0,0
PathOptimisers=RandomShortcut
ConfigurationShooter=RbprmShooter
PathValidation=RbprmPathValidation
ValidationTolerance=0.05
```

To modify or add environmental configurations, open `${CMAKE_INSTALL_PREFIX}/etc/gepetto-gui/rbprmEnvironments.conf` and write:
```
[Darpa]
RobotName=Darpa
Package=hpp-rbprm-corba
PackagePath=/local/anna/devel/install/share/hpp-rbprm-corba
URDFFilename=darpa
MeshDirectory=/local/anna/devel/install/share/

[Bauzil]
RobotName=Bauzil
Package=hpp-rbprm-corba
PackagePath=/local/anna/devel/install/share/hpp-rbprm-corba
URDFFilename=stair_bauzil
MeshDirectory=/local/anna/devel/install/share/
```
For full-body robots, open `${CMAKE_INSTALL_PREFIX}/etc/gepetto-gui/rbprmFullbodies.conf` and write:
```
[HYQ]
ModelName=hyq
RootJointType=freeflyer
Package=hyq_description
PackagePath=/local/anna/devel/install/share/hyq_description
URDFSuffix=
SRDFSuffix=
MeshPackage=hyq_description
MeshDirectory=/local/anna/devel/install/share/
nbSamples=20000,20000,20000,20000
cType=_3_DOF,_3_DOF,_3_DOF,_3_DOF
LegXY=0.02,0.02, 0.02,0.02, 0.02,0.02, 0.02,0.02
Resolution=0.05,0.05,0.05,0.05
LegIds=rfleg,lhleg,rhleg,lfleg
FirstJoints=rf_haa_joint,lh_haa_joint,rh_haa_joint,lf_haa_joint
LastJoints=rf_foot_joint,lh_foot_joint,rh_foot_joint,lf_foot_joint
Heuristics=manipulability,manipulability,manipulability,forward
Normals=0,1,0, 0,-1,0, 0,1,0, 0,-1,0
Offsets=0.,-0.021,0., 0.,0.021,0., 0.,-0.021,0., 0.,0.021,0.
DisableEffectorCollision=0,0,0,0
```
##### NOTE: Do not forget to replace `${CMAKE_INSTALL_PREFIX}` by a relevant path. Also make sure you have the package [hyq_description](https://github.com/iit-DLSLab/hyq-description) installed on your computer.
#### Loading plugins
To enable usage or rbprm-plugin, open `${CMAKE_INSTALL_PREFIX}/etc/gepetto-gui/settings.conf` and write:
```
[plugins]
libhpprbprmplugin.so=true
libhppwidgetsplugin.so=true
[pyplugins]
gepetto.gui.affordance=true
```
The plugins are looked for in the directory `${CMAKE_INSTALL_PREFIX}/lib/gepetto-gui-plugins`


