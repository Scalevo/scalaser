^^^^^^^^^
TESTCASES
^^^^^^^^^

(2015-06-08) - test automatic climb of curved stairs 
--------------------
Test Criteria:
	* Climb stairs without external alignment help
	* Recognize stairparameters and position on stairs
	* tested on CLA curved stairway

Results:
	* Even tough no extrapolation of the angle was implemented, going up the stairs worked fine.
	* Often a single, scattered measurement point on scan_1 messed up the matching process. -> filter which delets measurements in close proximity of the sensor was implemented
	* Single_Edge_Beta and Double_Edge_Beta performed simalar. Both worked about equally good.
	* Bug of changing beta angle according to direction of travel
		* while going up (dx > 0) kp needed to be positiv
		* while going down (dx < 0) negativ.
	* Turning Point:
		* the point around which the robot turns is fine while going up
		* while going down it lies way to far behind the chair, instead it should be on the front of the chair -> change by initializing dx_1 & dx_2 with a controlled offset or a constant alpha change once the alpha determination works
	* the wheelchair is usually quite unstable on the curved stairs this was tested -> Track adjustments might help out on this matter

(2015-xx-xx) - robustness test of stair recognition on stairs without solid backwalls
----------------------------------------------------------------------
Test Criteria:
	* Recognize stairparameters and position on stairs

Results:
	* 


(2015-06-03) - dynamic test of the controller on stairs
------------------------------------
Test Criteria:
	* Offset/Distance_Traveled [m]/[m] from straight line up/down during climbing straight stairs
	* Performance difference of going up and down

Results:
	* Wheelchair stays within +-1° straight on stairs
	* Offset was not measured during this first test

(2015-05-16) - static test of controller on stairs
-----------------------------------
Test Criteria:
	* has only been tested without driving on stairs due to safety measures

Results
	* Kp = 0.05 align the chair to -3° to 0° within 10 s.


(2015-05-16) - test multiple fmincon algorithms on their performance and accuracy
-------------
* sqp is the fastest with enough accuracy

(2015-05-16) - test multiple FoV configs on their performance and accuracy
-----------------------------------------------------------
Test Criteria:
	* At least three edges need to be within the FoV to guarantee a correct identification of the stair parameters
		* This is dependant mostly on the stair depth
		* An initial ranging scan with a big FoV can be done to reduce the FoV to its optimal size

Results:
	* 150 seems to be the sweetspot
	* more testing and calculations still needed

(2015-05-16) - variable track and stair visualization are working
--------------------------------------------------
Test Criteria:
	* Stairs grow dependant on distance traveled
	* Tracks change in regards to their position at the acctual wheelchair