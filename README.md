# RRT_Merci
RRT Implementation used for GRAIC Racing Competition

| Rapidly Exploring Random Tree (RRT)    | RRT*                                            |
|----------------------------------------|-------------------------------------------------|
| ![RRT Diagram](./doc/img/RRT_Tree.svg) | ![RRT Star Diagram](./doc/img/RRTStar_Tree.svg) |

## Overview
The RRT implementations are designed for 2D vehicle racing, containing

* Planners
    * Rapidly Exploring Random Trees (RRT)
    * RRT*
    * [TODO] Kinematic RRT
        * Dubins Path
    * [TODO] Dynamic RRT
        * LQR RRT?
* Obstacles
    * Circles
    * Rectangles 
    * Convex Polygons 
    * Parallel Curves 
* Smoothers
    * Random Path Smoothing (obstacle aware)
* Samplers
    * Uniform
    * [TODO] Sobol

## Citations

Algorithms were taken from their respective papers. Also, Pythonrobotics implementations were referenced.

### PythonRobotics
> Sakai, A., Ingram, D., Dinius, J., Chawla, K., Raffin, A., & Paques, A. (2018). Pythonrobotics: a python code collection of robotics algorithms. arXiv preprint arXiv:1808.10703.

### RRT
> LaValle, S. M., & Kuffner Jr, J. J. (2001). Randomized kinodynamic planning. The international journal of robotics research, 20(5), 378-400.

### RRT*
> Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. The international journal of robotics research, 30(7), 846-894.

## Python Bindings

[TODO] There will be python bindings to the rust implementations so it can be run on the GRAIC scenarios.