# Issues and Feature Ideas

## Issues
- Divergence when EE is close to goal.
    - Idea: Small tau + close to goal = small value = underflow?
- Last box appears to sometimes not have collision.
- Contact points switch between links and obstacles... how to make consistant?

## Review/Refactor/Optimize
- ContactPoint is heap allocated, create better structure to avoid this.
- B Matrix.
- Dynamic polymorphism for shapes rather than function overloading.
- Review structure and program flow.
    - Pay special attention to getters and whether they return references.
- Link safety distance so it only is changed in one place.
- Sparse matrix multiplication for jacobians.
- Review power of DualQuat.
- Motion Planner loop takes ~50-250 us.
    - SetJointAngles() takes ~40 us.
        - 3 us for kinematics, 37 us for generateContacts().
            - Majority of time spent in physx specific functions, not much room for optimization here.
                - Maybe I could use aggregates to better optimize broad phase as there is little difference between no contacts and contacts in time.
    - ScLERP takes ~10 us.
    - CollisionDisplacement takes ~0-200 us ().
        - LCP generation takes ~90% of this time.
            - Majority of time in generation split evenly between pseudoInv and assignment to M.
            - Suggest I rework to accomplish three things:
                1. Store column specific information as it is computed multiple times.
                2. Traverse the matrix for assignment in column-major order.
                3. Write my own pseudo-inverse function, or use QR/SVD decomposition, or some other least-squares solution.

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
