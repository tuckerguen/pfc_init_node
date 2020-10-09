## Running the Simulation and Node for Testing pfc_init Performance
### Configuring the Workspace
Option 1:
1. Clone the entire catkin workspace src directory into a new catkin workspace (find at: https://github.com/tuckerguen/pfc_init_catkin_ws).
2. Build the workspace, resolve errors
Option 2:
1. Clone the pfc_init project into an empty catkin workspace src directory
2. Resolve the missing dependencies manually
### Running the Simulation
3. catkin build the project, resolve errors
4. source devel/setup.bash from the root dir of the catkin workspace
5. In terminal run: roslaunch sim_gazebo test_pfc_init.launch (there may be errors but just check that the needle and background are spawned properly)
    * You can confirm that the simulation is running properly using (in a new terminal): "rosrun image_view image_view image:=/davinci_endo/left/image_raw"
    * Confirm that you see a needle on a cork board background and that the image is well lit.
### Running the pfc_init Node
6. In a new terminal cd into the workspace, source devel/setup.bash
7. rosrun pfc_init pfc_init_node
8. You may see various images during the process (these are mostly used for debugging). 
    * To continue, press any key
9. Once the program is done running you'll see an output of the results in the terminal as well as in image form