# Assignment3
## Jacobians computation for  FANUC R-2000iC/165F

### Robot description
- 6 degree of freedom manipulator with spherical wrist FANUC R-2000iC/165
- Construction weight - 1090kg
- Maximal weight of the load - 165kg
- Maximal reachable distance - 2655mm
![](fanuc.png)
<p align='center'> Geometrical data about the robot </p>
![](scheme.jpg)
<p align='center'> Kinematic scheme of the robot </p>

### Modeling
- Complete model for the robot:

  T = T<sub>base</sub> T<sub>z0</sub> R<sub>z</sub>(q1 + dq1)[T<sub>x1</sub> T<sub>y1</sub> R<sub>x1</sub> R<sub>y1</sub>]<sub>L1</sub> R<sub>x</sub>(q2 + dq2)[T<sub>z2</sub> T<sub>y2</sub> R<sub>z2</sub> R<sub>y2</sub>]<sub>L2</sub> R<sub>x</sub>(q3 + dq3)[T<sub> z3</sub> T<sub>y3</sub> R<sub>z3</sub> R<sub>y3</sub>]<sub>L3</sub> R<sub>y</sub>(q4 + dq4)[T<sub> z4</sub> T<sub>x4</sub> R<sub>z4</sub> R<sub>x4</sub>]<sub>L4</sub> R<sub>x</sub>(q5 + dq5)[T<sub> z5</sub> T<sub>y5</sub> R<sub>z5</sub> R<sub>y5</sub>]<sub>L5</sub> R<sub>y</sub>(q6 + dq6)[T<sub> z6</sub> T<sub>x6</sub> R<sub>z6</sub> R<sub>x6</sub>]<sub>L6</sub> T<sub>tool</sub>
- where q1, q2, q3, q4, q5, q6 - joint angles, dq1, dq2, dq3, dq4, dq5, dq6 - errors in joint angles, T<sub>z0</sub> - translation of first link (on l1, with error), T<sub>y1</sub> - translation of 2nd link (on l2, with error), T<sub>z2</sub> - translation of 3rd link (on l3, with error), T<sub>z3</sub>, T<sub>y3</sub> - translations of 4th link (on l4 by z and l5 by y, with error), other matrices stand for errors in links and joints mounts positions

- Move T<sub>z0</sub> to base, 6th link to tool, apply reduction rules and get the following irreducible model:

  T = T<sub>base</sub> R<sub>z</sub>(q1 + dq1)[T<sub>x1</sub> T<sub>y1</sub> R<sub>y1</sub>]<sub>L1</sub> R<sub>x</sub>(q2 + dq2)[T<sub>z2</sub> R<sub>z2</sub> R<sub>y2</sub>]<sub>L2</sub> R<sub>x</sub>(q3 + dq3)[T<sub> z3</sub> T<sub>y3</sub> R<sub>z3</sub>]<sub>L3</sub> R<sub>y</sub>(q4 + dq4)[T<sub> z4</sub> T<sub>x4</sub> R<sub>z4</sub>]<sub>L4</sub> R<sub>x</sub>(q5 + dq5)[T<sub> z5</sub> R<sub>z5</sub>]<sub>L5</sub> R<sub>y</sub>(q6 + dq6)T<sub>tool</sub>

- Calibration procedure:
  1. Initially set all errors to 0
  2. Generate 30 random configurations
  3. Compute the expected (calculated from estimated model) tools positions and transformation matrix for robot (for 3 tools) and real (from the model which include real error values)
  4. From estimated transformation matrix get position and orientation of end effector relative to base, make skew-symmetric matrix for position vector
  5. Estimate base and tools positions by formula (where [~p] - skew symmetric matrix from 4, Δp<sub>i</sub> - difference between true and estimated position value for all tools from ith measured configuration, A has dimensions 9x15 since there are 3 tools):
  ![](base.png)
  6. Compute (using numerical method) Jacobians of matrix T<sub>base</sub>T<sub>robot</sub>T<sub>tool</sub> by error parameters
  7. Compute new parameters for robot transformation matrix by formula (m = 30 - num of experiments, j = 3 - num of tools):
  ![](pi.png)
  8. Repeat 2 - 7 during some number of iterations

### Github link
- https://github.com/jenamax/Introdution-to-Robotics/tree/master/Assignment4
