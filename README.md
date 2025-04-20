# floating base model

## Introduction
This project provides a [ocs2](https://github.com/leggedrobotics/ocs2) model of dynamics (system flow map) for legged robots using floating base model [1] [2]:
```math
\dot{\boldsymbol{x}} = \boldsymbol{f}(t, \boldsymbol{x}, \boldsymbol{u}, \boldsymbol{d})
```
where:
- t: time
- $\boldsymbol{x}$: system state vector
```math
\boldsymbol{x} =
\begin{bmatrix}
\boldsymbol{v}^B_B\ \\
\boldsymbol{\omega}^B_B\ \\
\boldsymbol{r}^0_B \\
\boldsymbol{q}_E \\
\boldsymbol{q}_j 
\end{bmatrix}
```
- $\boldsymbol{u}$: input state vector
```math
\boldsymbol{u} =
\begin{bmatrix}
\boldsymbol{f}_{ext_1} \\
... \\
\boldsymbol{f}_{ext_{n}} \\
\boldsymbol{f}_{ext_{n+1}} \\
\boldsymbol{\tau}_{ext_1} \\
... \\
\boldsymbol{f}_{ext_{n+m}} \\
\boldsymbol{\tau}_{ext_m} \\
\dot{\boldsymbol{q}}_j
\end{bmatrix}
```
- $\boldsymbol{d}$: disturbance vector
```math
\boldsymbol{d} =
\begin{bmatrix}
\boldsymbol{f}_{ext_{B}} \\
\boldsymbol{\tau}_{ext_B} 
\end{bmatrix}
```

Equations of dynamics (System Flow Map)
```math
\dot{\boldsymbol{x}} = 
\begin{bmatrix}
\frac{d\boldsymbol{v}^B_B}{dt}\ \\
\frac{d\boldsymbol{\omega}^B_B}{dt}\ \\
\frac{d\boldsymbol{r}^0_B}{dt} \\
\frac{d\boldsymbol{q}_E}{dt} \\
\frac{d\boldsymbol{q}_j}{dt} 
\end{bmatrix} = \begin{bmatrix}
\boldsymbol{aba}_{B_v}(\boldsymbol{q}, \boldsymbol{v}, \boldsymbol{f}_{ext}, \boldsymbol{\tau}_{ext}, \boldsymbol{d}) +  \boldsymbol{w}^B_B \times \boldsymbol{v}^B_B \\
\boldsymbol{aba}_{B_w}(\boldsymbol{q}, \boldsymbol{v}, \boldsymbol{f}_{ext}, \boldsymbol{\tau}_{ext}, \boldsymbol{d}) \\
\boldsymbol{R}^0_B(\boldsymbol{q}_E)\boldsymbol{v}^B_B \\
\boldsymbol{E}(\boldsymbol{q}_E)\boldsymbol{w}^B_B \\
\dot{\boldsymbol{q}}_j
\end{bmatrix}
```
```math
\boldsymbol{aba}_{B}(\boldsymbol{q}, \boldsymbol{v}, \boldsymbol{f}_{ext}, \boldsymbol{\tau}_{ext}, \boldsymbol{d}) =
\boldsymbol{M}^{-1}_B \big(-\boldsymbol{C}(\boldsymbol{q}, \boldsymbol{v}) \boldsymbol{v} - \boldsymbol{G}(\boldsymbol{q}) + \sum_{i \in C} \boldsymbol{J}^T_{B, i}\boldsymbol{F}_{ext_i} + \boldsymbol{F}_{ext_B} \big)
```
```math
\boldsymbol{q} =
\begin{bmatrix}
\boldsymbol{r}^0_B \\
\boldsymbol{q}_Q \\
\boldsymbol{q}_j 
\end{bmatrix}
,

\boldsymbol{v} =
\begin{bmatrix}
\boldsymbol{v}^B_B \\
\boldsymbol{\omega}^B_B \\
\dot{\boldsymbol{q}_j}
\end{bmatrix}
,

\boldsymbol{F}_{ext_i} =
\begin{bmatrix}
\boldsymbol{f}_{ext_i} \\
\boldsymbol{\tau}_{ext_i}
\end{bmatrix}
,

\boldsymbol{F}_{ext_B} =
\begin{bmatrix}
\boldsymbol{f}_{ext_B} \\
\boldsymbol{\tau}_{ext_B}
\end{bmatrix}
```

where:
- $\boldsymbol{v}^B_B$: base linear "classical" velocity expressed in base frame of reference ($B$) [0]
- $\boldsymbol{\omega}^B_B$: base angular "classical" velocity expressed in base frame of reference ($B$) [0]
- $\boldsymbol{r}^0_B$: base position from inertial frame expressed in inertial frame of reference ($0$)
- $\boldsymbol{q}_E$: base orientation from inertial frame defined in [ZYX Euler angles](https://web.mit.edu/2.05/www/Handout/HO2.PDF)
- $\boldsymbol{q}_Q$: base orientation from inertial frame defined in [quaterions](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation)
- $\boldsymbol{q}_j$: actuated joint positions
- $\dot{\boldsymbol{q}_j}$: actuated joint velocities
- $\boldsymbol{q}$: generalized positions of pinocchio multibody system 
- $\boldsymbol{v}$: generalized velocities of pinocchio multibody system
- $\boldsymbol{R}^0_B(\boldsymbol{q}_E)$: base to inertial frame rotation matrix
- $\boldsymbol{E}(\boldsymbol{q}_E)$: matrix that maps base angular "classical" velocity expressed in base frame of reference to derivative of ZYX Euler angles
- $\boldsymbol{J}_{B, i}$: robots end-effector "spatial" jacobian matrix [0]
- $\boldsymbol{f}_{ext}$: external force acting on robot end-effectors
- $\boldsymbol{\tau}_{ext}$: external torque acting on robot end-effectors
- $\boldsymbol{F}_{ext_i}$: external "spatial" force acting on robot end-effectors expressed in inertial frame of reference [0]
- $\boldsymbol{F}_{ext_B}$: external disturbance "spatial" force acting on robot base, expressed in base frame of reference [0]
- $\boldsymbol{M}_B$: $6 \times 6$ "spatial" inertia matrix called "Locked Spatial Inertia Matrix" [0]
- $\boldsymbol{aba}_{B}(...)$: equation for solving "spatial" base acceleration ($v$ is linear and $\omega$ is angular part of equation) [0]

## ROS 2 versions
- All 

## Dependencies
- [pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [ocs_ros2](https://github.com/BartlomiejK2/ocs2_ros2):
  - ocs2_pinocchio_interface
  - ocs2_centroidal_model
  - ocs2_robotic_tools
#### :warning: IMPORTANT: ocs2 is big monorepo, first clone and build all ocs2 packages in you'r workspace. It can take up to an hour.

## Installation 
1. Clone repo to your workspace:
```bash
git clone https://github.com/KNR-PW/LRT_floating_base_model.git
```
2. Install dependencies in workspace:
```bash
rosdep install --ignore-src --from-paths . -y -r
```
3. Build:
```bash
colcon build --packages-select floating_base_model
```
## API Documentation
1. Generate local (in workspace) API documentation using `ros2doc`:
```bash
rosdoc2 build --package-path src/floating_base_model
```
2. Show documentation in browser e.g. firefox:
```bash
firefox docs_output/floating_base_model/index.html
```

## Contributing
1. Change code.
2. Pass all unit tests:
```bash
colcon build --packages-select floating_base_model
colcon test  --packages-select floating_base_model
```
3. Add clear description what changes you've made in pull request.

## Reference 
[0] R. Featherstone, “Rigid Body Dynamics Algorithms“, November, 2007, doi: 10.1007/978-1-4899-7560-7.

[1] R. Grandia, F. Jenelten, S. Yang, F. Farshidian, and M. Hutter, “Perceptive Locomotion through Nonlinear Model
Predictive Control”, (submitted to) IEEE Trans. Robot., no. August, 2022, doi: 10.48550/arXiv.2208.08373.

[2] J. P. Sleiman, F. Farshidian, M. V. Minniti, and M. Hutter, “A Unified MPC Framework for Whole-Body Dynamic
Locomotion and Manipulation”, IEEE Robot. Autom. Lett., vol. 6, no. 3, pp. 4688–4695, 2021, doi:
10.1109/LRA.2021.3068908.

[3] S. Stramigioli, V. Duindam, G. van Oort, and A. Goswami, “Compact
Analysis of 3D Bipedal Gait Using Geometric Dynamics of Simplified
Models,” in Robotics and Automation, 2009. ICRA ’09. IEEE International Conference on, May 2009, pp. 1978–1984.
