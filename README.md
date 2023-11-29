# UnrealSim
First Things to Know:
For this Project UnrealSim, main branch is working all the platforms(windows, mac, ubuntu) but we use urdf parser to simulate all the objects(simple and articualted) and robots in unreal engine. But here we have to manually hardcode the objections positions and other stuff in the code

sdf_integration branch is useful in this case, where we can add mention the complete scene in an sdf file where we can simulate all the objects but for robots we are still using urdf parser.

but the disadvantage here currently is our sdf parser is only working in windows. The issues we faced were.


Setting up the Project:
This Unreal Project has 2 modules, named "UnrealSim" and "SDF". The Second Module "SDF" is an external module which uses SDFLibrary which is a third party 
c++ library, which again needs multiple dependencies to install first.
The Following Dependencies are needed for SDF Library to work.
1. Ignition Math
2. Ruby
3. boost-cpp
4. tinyxml
5. urdfdom

Take a look at the below commands to get the dll files for windows. You have to follow these similar commands to get the project working in your windows machine
 cd sdformat
 mkdir build
 cd build
 ls
 cmake .. -DCMAKE_INSTALL_PREFIX=./install
 make install

this way you have to install all the c++ libraries in the windows system.

After Setting up the libraries you have to open visual studio and then open unreal engine to open the project.

sdf files are located at UnrealSim/Content/SceneDescriptin/sdf/ and for every new object that you add you need to add, add the meshes at 
UnrealSim/Content/SceneDescription/meshes/<objectName>/   

this way we can simulate the objects in unreal engine.


