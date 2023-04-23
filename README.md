# Space Exploration
Implementation of several algorithms for space and unknown terrain exploration.

## Table of Contents
- Path finding algorithms
  - [D* Lite](#dlite) 
  - [A*](#a) 
- Frontier detection algorithms 
- Path Smoothing


## D* Lite

Path finding algorithms specifically designed for unknown environmments. Its efficiency lays in the possibility to find a new path without entirely recompute the whole path from goal o source.

Algorithm described in _Koenig, S., & Likhachev, M. (2005). Fast replanning for navigation in unknown terrain. IEEE Transactions on Robotics, 21(3), 354–363._

<img src="https://github.com/mattianeroni/space-exploration/blob/main/images/dstar.gif" width="50%" height="50%">

                                                                                                                                             


## A*

Efficient path finding algorithm generally described as a natural evolution of Djikstra algorithm after the introduction of the direction's concept.

Algorithm described in _Hart, P. E.; Nilsson, N.J.; Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths". IEEE Transactions on Systems Science and Cybernetics. 4 (2): 100–7._

<img src="https://github.com/mattianeroni/space-exploration/blob/main/images/astar.gif" width="50%" height="50%">




## Frontier Detection

Efficient frontier detection algorithm inspired by the Expanding Wavefront Frontier Detection algorithm presented in _Quin, P., Alempijevic, A., Paul, G., & Liu, D. (2014, January). Expanding wavefront frontier detection: An approach for efficiently detecting frontier cells. In Australasian Conference on Robotics and Automation, ACRA._, but made faster through the implementation of an [RTree](https://it.wikipedia.org/wiki/R-tree) able to detect the only frontiers expanded by the robot at each iteration.

<img src="https://github.com/mattianeroni/space-exploration/blob/main/images/frontier.png"  width="50%" height="50%">




## Path Smoothing

Two path smoothing algorithms.


