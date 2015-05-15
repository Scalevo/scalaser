^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scalaser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO
------------------

* make fov_s, fov_d, dzi, phi0 dependant of the lambda position received from the MyRIO
* create launch-file to start the full package node
* test multiple fmincon algorithms on theire performance and accuracy
* plot beta and error to file and afterwards in matlab
* if beta values are different for to long resinitialize computation
* Fix inconsistent performance of the angle node. It ranges from 1 Hz to 4 Hz. If a rviz is started the frequency takes a hit, however once it's closed again the node wont performe as well as before.
* try to reduce FoV to increase performance.
* make fov_s & fov_d eiditable during runtime

NICE TO HAVE
------------------
* fix track marker
* make stair grow and shrink dependant on distance traveled
* (change the initialization of v0 from within the angle constructor to the matching constructor since startvalues for both sides are identical)
* change tf static to tf2 static.

0.0.6 (2015-5-13)
-----------------
* reset beta_old and beta_new before restarting service
* added complete wheelchair model to visualtization
* created launch-file to start the visualization (stair_viz.launch)


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
