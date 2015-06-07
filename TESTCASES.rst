^^^^^^^^^
TESTCASES
^^^^^^^^^

* test old beta control
* turn on stairs
* test new beta control

* beta vorzeichen problem hochfahren kp - runterfahren kp +
* laser scan filtern auf komische werte


(2015-xx-xx) test automatic climb of curved stairs 
--------------------
Test Criteria:
	* Recognize stairparameters and position on stairs
	* Here not all 811 scan points might be used, a fix of this might be needed therefore
	* 

Results:
	*

(2015-xx-xx) robustness test of stair recognition on stairs without solid backwalls
----------------------------------------------------------------------
Test Criteria:
	* Recognize stairparameters and position on stairs
	* Here not all 811 scan points might be used, a fix of this might be needed therefore
	* 

Results:
	* 


(2015-06-03) dynamic test of the controller on stairs
------------------------------------
Test Criteria:
	* Offset/Distance_Traveled [m]/[m] from straight line up/down during climbing straight stairs
	* Performance difference of going up and down

Results:
	* Wheelchair stais within +-1° straight on stairs
	* Offset was not measured during this first test

(2015-05-16) static test of controller on stairs
-----------------------------------
Test Criteria:
	* has only been tested without driving on stairs due to safety measures

Results
	* Kp = 0.05 align the chair to -3° to 0° within 10 s.


(2015-05-16) test multiple fmincon algorithms on their performance and accuracy
-------------
* sqp is the fastest with enough accuracy

(2015-05-16) test multiple FoV configs on their performance and accuracy
-----------------------------------------------------------
Test Criteria:
	* At least three edges need to be within the FoV to guarantee a correct identification of the stair parameters
		* This is dependant mostly on the stair depth
		* An initial ranging scan with a big FoV can be done to reduce the FoV to its optimal size

Results:
	* 150 seems to be the sweetspot
	* more testing and calculations still needed

(2015-05-16) variable track and stair visualization are working
--------------------------------------------------
Test Criteria:
	* Stairs grow dependant on distance traveled
	* Tracks change in regards to their position at the acctual wheelchair