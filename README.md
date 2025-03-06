# floating base model

## Introduction
This project provides a [ocs2](https://github.com/leggedrobotics/ocs2) model of dynamics (system flow map) for legged robots using floating base model [1] [2]:
```math
\dot{\boldsymbol{x}} = \boldsymbol{f}(t, \boldsymbol{x}, \boldsymbol{u})
```
where:
- t: time
- $\boldsymbol{x}$: system state vector:
```math
\boldsymbol{x} =
\begin{bmatrix}
\boldsymbol{v}^B_B\ \\
\boldsymbol{w}^B_B\ \\
\boldsymbol{r}^0_B \\
\boldsymbol{q}_E \\
\boldsymbol{q}_j 
\end{bmatrix}
```
- $\boldsymbol{u}$: input state vector:
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


Equations of dynamics (System Flow Map):
```math
\dot{\boldsymbol{x}} = 
\begin{bmatrix}
\frac{d\boldsymbol{v}^B_B}{dt}\ \\
\frac{d\boldsymbol{w}^B_B}{dt}\ \\
\frac{d\boldsymbol{r}^0_B}{dt} \\
\frac{d\boldsymbol{q}_E}{dt} \\
\frac{d\boldsymbol{q}_j}{dt} 
\end{bmatrix} = \begin{bmatrix}
\boldsymbol{aba}_B(\boldsymbol{q}, \dot{\boldsymbol{q}}, \boldsymbol{f}_{ext}, \boldsymbol{\tau}_{ext})[0:2] +  \boldsymbol{w}^B_B \times \boldsymbol{v}^B_B \\
\boldsymbol{aba}_B(\boldsymbol{q}, \dot{\boldsymbol{q}}, \boldsymbol{f}_{ext}, \boldsymbol{\tau}_{ext})[3:5] \\
\boldsymbol{R}^0_B(\boldsymbol{q}_E)\boldsymbol{v}^B_B \\
\boldsymbol{E}(\boldsymbol{q}_E)\boldsymbol{w}^B_B \\
\dot{\boldsymbol{q}}_j
\end{bmatrix}
```
```math
\boldsymbol{aba}_B(\boldsymbol{q}, \dot{\boldsymbol{q}}, \boldsymbol{f}_{ext}, \boldsymbol{\tau}_{ext}) =
\boldsymbol{M}^{-1}_B \big( - \boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}} - \boldsymbol{G}(\boldsymbol{q}) + \sum_{i \in C} \boldsymbol{J}^T_{B, i}\boldsymbol{f}_{ext_i} + \sum_{i \in C}\boldsymbol{\tau}_{ext_i} \big)
```

where:
- $\boldsymbol{v}^B_B$: base linear "classical" velocity expressed in base frame of reference ($B$) [0]
- $\boldsymbol{w}^B_B$: base angular "classical" velocity expressed in base frame of reference ($B$) [0]
- $\boldsymbol{r}^0_B$: base position from inertial frame expressed in inertial frame of reference ($0$)
- $\boldsymbol{q}_E$: base orientation from inertial frame defined in [ZYX Euler angles](https://web.mit.edu/2.05/www/Handout/HO2.PDF) 
- $\boldsymbol{q}_j$: actuated joint positions
- $\boldsymbol{q}$: generalized positions of pinocchio multibody system 
- $\boldsymbol{q}$: generalized velocities of pinocchio multibody system
- $\boldsymbol{R}^0_B(\boldsymbol{q}_E)$: base to inertial frame rotation matrix
- $\boldsymbol{E}(\boldsymbol{q}_E)$: matrix that maps base angular "classical" velocity expressed in base frame of reference to derivative of ZYX Euler angles
- $\boldsymbol{J}_{B, i}$: robots end-effector jacobian matrix
- $\boldsymbol{f}_{ext}$: external force acting on robot end-effectors
- $\boldsymbol{\tau}_{ext}$: external torque acting on robot end-effectors
- $\boldsymbol{M}_B$: $6 \times 6$ "spatial" inertia matrix called "Floating Base Locked Spatial Inertia Matrix" [0]
- $\boldsymbol{aba}_Bv(...)$, $\boldsymbol{aba}_Bw(...)$: equation for solving "spatial" base acceleration [0]

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
