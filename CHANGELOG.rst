^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scalaser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO
------------------
* find error that causes beta to randomly jump ~20° (probably caused by improper phase offset)
  - solution_1: tracking of the motor encoders helps to determine if dx makes sense and changes dx accordingly
  - solution_2: using the fminsearch results from the other side as start values reduces chances of dx beeing phaseoffset
  - solution_3: filter which checks previous dx value and beta value and compares them to check if the difference make any sense
* implement service which starts the beta computation
* make fov_s,fov_d,dzi,phi0 dependant of the lambda position received from the MyRIO

NICE TO HAVE
------------------
* change the initialization of v0 from within the angle constructor to the matching constructor since startvalues for both sides are identical
* only compute angle if pointclouds are published
* add complete wheelchair model to visualtization
* make stair grow and shrink dependant on distance traveled

0.0.3 (2014-05-10)
------------------
* fminsearch was replaced with fmincon to set boundary conditions
* filter for beta and se_r added to only publish if values are below a threshold
* messages are now saved and computed at a synchronized and steady rate using a timer
* desired velocities are sent to MyRIO to align the chair on the stairs automatic. Kp can also be changed using the parameter server.
* using .mat-files for data transfer to MATLAB has decreased the computation time of one angle computation from ~1 second to ~0.2 seconds.
* README has been created

0.0.2 (2014-04-30)
------------------
* markers to represent the wheelchairs position on the stairs have been implemented
* fov_s,fov_d,dzi,phi0 can now be set using ROS parameters within the launch file

0.0.1 (2015-04-23)
------------------
* beta and result publisher are now working
* fminsearch is now working
* initial commit
* Contributors: Miro Voellmy
