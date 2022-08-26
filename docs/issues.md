# Issues and Feature Ideas

## Issues
- Divergence when EE is close to goal.
    - Idea: Small tau + close to goal = small value = underflow?
- Last box appears to sometimes not have collision.
- Contact points switch between links and obstacles... how to make consistant?

## Review/Refactor/Optimize
- B Matrix.
- Dynamic polymorphism for shapes rather than function overloading.
- Review structure and program flow.
    - Pay special attention to getters and whether they return references.
- Link safety distance so it only is changed in one place.
- Sparse matrix multiplication for jacobians.
- Review power of DualQuat.

## New Features
- Add gripper bodies and joints.
    - Do I need to switch to a KinematicTree?
- Add metric to determine if the grasp is possible.
    - Distance + Normals.
    - Take the normals from the object to both of the grippers, and if they lie within a certain cone, it is a good grasp.

## LCP Solver
- Look into IGS solver.
- Switch to OOP.
- Add proper tests.
- Can choose pivots with collision information. Closest points tell us next best pivot?
