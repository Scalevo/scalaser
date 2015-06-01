Parameters needed to detect curved stairs
Angle between middle steps


A way to track the edges is neccessary since the determination over the dx of each side does not work any more. A reset of the dx might be neccesary after dx reaches a certain threshold. An option would be to use the smallest difference between the modulo and/or original value of dx to track the distance traveled on the stairs.  The laser on the outside of the stairs will always measure more dx over the course of the stairs. A normalized dx in dependancy of the step angle is needed to compensate for said unequality.
If we assume, that the stair curve is equal accros the whole field of view we can determine the angle (alpha) between the individual edges out of the step diagonal difference between the outer and inner scan.
The equation goes as following:

cos(beta) * outer_diagonal = cos(beta) * inner_diagonal + tan(alpha) * distance_between_sensors / cos(beta)

If this is solved for beta using non linear solver we can adjust ourselves on the stairs as required.
The challenge lies in the fact that alpha needs to be determined out of LIDAR data without knowing beta beforehand. If the influence of beta can be represented using a constant alpha can be normalized and later used for actual computation of beta.

outer_diagonal = inner_diagonal + tan(alpha) * distance_between_sensors  // formula on curved stairs if the wheelchair is parallel to the shortest distance between the two edges.

Going up curved stairs the chair can be oriented on the middle value of two diagonals. The problem hereby lies in the fact, that the change from edge one 1 & 2 to edge 2 & 3 leads to stepwise adjustment of desired angle around we adjust the chair.
A continously changing "sollwinkel" would be ideal and lead to a more smooth ride up the curved stair.
The distance traveled (dx) can be used to determin the desired angle if alpha is constant over the whole stairs.

The angle we want to minimize is computed out of the step parameters which are scaled depending on the angle of the wheelchair on the curved stairs.


   //   |
  //	|
 //		|