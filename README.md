# UnrealSim
## Project Description: 
This is a VR Simulation of [Human-Guided Planning for Complex Manipulation Tasks Using the Screw Geometry of Motion](https://irsl-sbu.github.io/Human-Guided-Planning/)
 
This project is a VR simulation of a Kitchen environment. The simulation is built using Unreal Engine 4.27.4 and the VR headset used is Oculus Quest. The simulation is built using the Unreal Engine's C++ API. 
To Navigate in the sim, use the thumsticks on both controllers to get to desired position. We can interact the objects in the simulation using the grip button on the Right VR controller.
Grab on the handles of cabinets/fridge doors/ drawers to open them. Grab on the objects to move them around.

Steps to run the VR simulation:
1. Install Unreal Engine 4.27.4
2. Install the Oculus App. Follow the instructions here: https://www.meta.com/help/quest/articles/headsets-and-accessories/oculus-rift-s/install-oculus-pc-app/
3. Clone this repository
4. Create a solution file for Visual Studio 2019 by right clicking on the .uproject file and selecting "Generate Visual Studio project files"
5. Make sure to activate oculus link on your headset and connect it to your computer before luanching the project in Unreal Engine
6. Once the oculus device is connected, click on the play(change it to VR Preview) button in Unreal Engine to launch the simulation
7. Make Changes to scene2.sdf file in VRSim\Content\SceneDescription\sdf folder to make changes to the scene

