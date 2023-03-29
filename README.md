# ReedsSheppCurve-Application-Improvement
This demo represent how to use the ReedsShepp Curve to plan the trajectory
from one pose to another pose on the no obstacles map

This demo extract and reorganize the relevant code that is from a hybrid
A star algorithm demo (download from the https://github.com/wanghuohuo0716/hybrid_A_star)

At the same time，make the 7 modifications：

1）extract the code that complete the "CSCC" calculation，and create a single
  function to accommodate them. that make the struct of the program more clarity

2) when select the trajectory from the several candidate trajectory，not only
   consider the length of trajectory，but also focus on the back driving，
   the switch from forward to backward，and so on. make use the cost function
   to punish the focused vehicle behavior and select the optimization trajectory

3）when use the above modifications，the limit of trjectory direction at every
   function that accomplish the calculation in the reference paper will decrease
   the optional trajectory number，so delete the limit

4）after applying the 3), some candidate trajectory conld not reach the goal
   so, add a funcion to estimate if the trajectory can make the vehicle
   reach the goal, if not, delete the trajectory

5) the calculation of some code accomplished can get right result, but the specific
   formula is different from the formula of reference paper. And the formula in
   reference paper can also get the right result. if the above situation appear
   use the formula in the reference paper.

6) regrading to the "CCCC" type trajectory, when generate the "path", the sign
   of "t", "u", "v" (which is subtrajectory length) have some error, so
   make correction

7) change the path type from the class type to the struct type

reference paper：OPTIMAL PATHS FOR A CAR THAT GOES BOTH FORWARDS AND BACKWARDS
author：J. A. REEDS AND L. A. SHEPP

reference book: Planning Algorithms (by LaValle), page 884
