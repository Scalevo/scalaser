^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Approaches to Climb Curved Stairs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Some way to track the individual edges is neccessary since the determination over the dx of each side does not work any more. The laser on the outside of the stairs will always measure more dx over the course of the stairs.

Going up curved stairs the chair can be oriented on the middle value of two diagonals. The change from edge one 1 & 2 to edge 2 & 3 leads to stepwise adjustment of desired angle around we adjust the chair.
A continously changing "sollwinkel" would be ideal and lead to a more smooth ride up the curved stair. The distance traveled (dx) can be used to determin the desired angle via extrapolation if alpha is constant over the whole stairwayxc.

R-CUTPOINT APPROACH - 2015-06-03 (Double Edge)
----------------------------------------------
Alpha and Beta can be computed very easy if two edges and their position in regards to the chair are known. Check the manual drawing.
The edges can be found with the initialization method of the closest edge approach and need to be updated the same way.

CLOSEST EDGE APPROACH - 2015-06-03 (Single Edge) 
------------------------------------------------
The so called closest edge approach leads to an orientation of the wheelchair around an individual edge around which the the wheelchair is oriented. 
For this to work however the boundaries the initialization of dx need to lead to a dx, which describes the edge in a precise way without the possibility of a phase offset of our template.

The initialization of dx could look as follows:

Boundry initalization
	dx_1[-0.2 + c, .2 + c] 

	dx_2[dx_1 + -0.2, dx_1 + .2]

Setting these boundaries leads to finding the edge which is the closest to the sensor. If we can straighten ourself on this edge, the wheelchair can drive up safely. By changing the variable c we can adjust around which edge the wheelchair adapts himself on the stairs. This could either be the one on the front, in the middle or on the back. The ideal c for going up might even be different from the one going down the stairs. This has to be determined during testing.


If condition to reset dx, once it isn't the closest edge anymore. Instead of checking for edges, which are within the boundaries of the tracks we simply always check for the closest edge directly below our sensor. This can be done by checking the diagonal of the stair and comparing it to dx.

The reset however must occur on both sides simultaneously.

Boundry Update Condition
	if(diag_1 - abs(dx_1) < diag_1 / 2 || diag_2 - abs(dx_2) < diag_2 / 2) reinitalizeDx();

OUTDATED ALPHA/BETA APPROACH - 2015-05-31
-----------------------------------------
If we assume, that the stair curve is equal accros the whole field of view we can determine the angle (alpha) between the individual edges out of the step diagonal difference between the outer and inner scan.

Formula to determine alpha:
	cos(beta) * outer_diagonal = cos(beta) * inner_diagonal + tan(alpha) * distance_between_sensors / cos(beta)

If this is solved for beta using non linear solver we can adjust ourselves on the stairs as required.
The challenge lies in the fact that alpha needs to be determined out of LIDAR data without knowing beta beforehand.
I am confident, that this can be achieved however.

If it can not be achieved the following solutions might also work:

Alpha can be determined if the wheelchair is aligned properly on the stair allready and the influend of beta is therefore negligable.

Formula on curved stairs if the wheelchair is parallel to the shortest distance between the two edges
	outer_diagonal = inner_diagonal + tan(alpha) * distance_between_sensors

* By running a intialization scan series with different betas on the stairs we now know the 3D stair model. Using the smallest diagonal to compute the reoccuring alpha using the following equation.
* By running the closest edge approach the wheelchair can align itself.
The angle we want to minimize is computed out of the step parameters which are scaled depending on the angle of the wheelchair on the curved stairs.