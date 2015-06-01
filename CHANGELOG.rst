^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scalaser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO
------------------

* develope algorithm to compute curved stairs
* Fix inconsistent performance of the angle node. It ranges from 1 Hz to 4 Hz. If a rviz is started the frequency takes a hit, however once it's closed again the node wont performe as well as before.
* use IMU values to set guess for beta threshold value -> not sure if needed, since the wheelchair never comes close to a beta=~24°
* if the sensors are used outside, it can occur that the pointcloud size gets reduced which leads to a shift of FoV of each sensor -> phase offset
* !!! a restart of the service now reinitializes fov_s, fov_d, phi0, dzi from the parameter server !!! This is bugged. Fixme plz.
* Why does a reinitialization of the parameters cause such a huge mess


NICE TO HAVE
------------------
* add initialization for start and end of stairs to determine when to stop the stair model
* add timestamp to published messages
* make fov_s, fov_d dependant of the lambda position received from the MyRIO
* make stair growth dependant of fov_s
* define beta threshold in launch file
* (change the initialization of v0 from within the angle constructor to the matching constructor since startvalues for both sides are identical)
* (change tf static to tf2 static)

TESTS
------------------
* test controler on stairs
  - has only been tested static without driving on stairs due to safety measurs
  - Kp = 0.05 align the chair to -3° to 0° within 10 s.
  - more testing still needed
  - ...
* test multiple fmincon algorithms on their performance and accuracy
  - sqp is the fastest with enough accuracy
* test multiple FoV configs on their performance and accuracy
  - 150 seems to be the sweetspot
  - more testing still needed
  - ...
* variable track model works
* variable stair model works

0.0.10 (2015-6-2)
-----------------
* fixed a bug, which caused the stair tf to get all messed up
* realized, that the pointcloud vector isn't always of size 811 and thus the method of reduzing its fov doesn't work that robust

0.0.9 (2015-5-19)
-----------------
* created launch-file to start the wheelchair alignment for testing
* updated README
* added tests section to changelog

0.0.8 (2015-5-18)
-----------------
* robot model of the wheelchair has been created in the package SCALEBOT 
* dzi, phi0 are now computed out of the lambda positions received from the MyRIO -> this might only be needed during initialization to save computation time.
* if beta values are unusual for a number of computations, matching gets reinitialized
* fov_s & fov_d are editable before restart of the service

0.0.7 (2015-5-15)
-----------------
* changed the velocity publisher from Float64MultiArray to String
* now plots beta after service is finished
* replaced all 3.14 values with PI
* service now only starts the initialization if pointclouds are published

0.0.6 (2015-5-13)
-----------------
* resets beta_old and beta_new before restarting service
* added complete wheelchair model to visualtization
* created launch-file to start the visualization (stair_viz.launch)
* added rviz file
* stair visualization now grows dependant on distance traveled on the stairs

0.0.5 (2015-5-12)
-----------------
* now sets boundary constrains of dx to "dx_old ± (stair-diagonal)/2" to stop phase offset completely
* changed initializer vector v0 from <vector> to Eigen
* now initializes v0 and boundry contraints with the start of the ros service. The result vector of the first matching are used as the start values for the first matching of the other side.
* found error that causes beta to randomly jump ~20° caused by phase offset of dx_1 and dx_2

  Approach:
  - solution_1: tracking of the motor encoders helps to determine if dx makes sense and changes dx accordingly
  - solution_2: using the fminsearch results from the other side as start values reduces chances of dx beeing phaseoffset
  - solution_3: filter which checks previous dx value and beta value and compares them to check if the difference make any sense
  - solution_4: use boundary constrains of dx to "dx_old ± (stair-diagonal)/2" to stop wrong phase offset
  
  After succesfull implentation of the above methods, the modulo constraint of beta can be removed to allow for wider angle determination range.
  
  - solution_1: discarded     - since it would make the program dependant on correct encoder values
  - solution_2: implemented   - reduced the chance of getting phaseoffset dx values at the start of the programm to a very low percentage.
  - solution_3: (implemented) - only implemented for beta. Filters betas which are more than 10° bigger or smaller than the previous one.
  - solution_4: implemented   - reduced the phase offset of dx pretty good

  As it turned out was the modulo constraint of beta the source of the randomly occuring phase offset at around 40 seconds in the bag file "up_turn_down_pointcloud.bag". It happend always when one dx was close to the modulo threshold while the other was bigger than the threshold. This lead to a big difference between dx_1 and dx_2 which led to a unexpected big beta.

0.0.4 (2015-5-11)
------------------
* computes the angle only if pointclouds are published
* implemented service which starts the wheelchair alignment

0.0.3 (2015-05-10)
------------------
* fminsearch was replaced with fmincon to set boundary conditions
* filter for beta and se_r added to only publish if values are below a threshold
* messages are now saved and computed at a synchronized and steady rate using a timer
* desired velocities are sent to MyRIO to align the chair on the stairs automatic. Kp can also be changed using the parameter server.
* using .mat-files for data transfer to MATLAB has decreased the computation time of one angle computation from ~1 second to ~0.2 seconds.
* README has been created

0.0.2 (2015-04-30)
------------------
* markers to represent the wheelchairs position on the stairs have been implemented
* fov_s,fov_d,dzi,phi0 can now be set using ROS parameters within the launch file

0.0.1 (2015-04-23)
------------------
* beta and result publisher are now working
* fminsearch is now working
* initial commit
* Contributors: Miro Voellmy