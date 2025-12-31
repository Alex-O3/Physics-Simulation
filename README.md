This is a Physics Simulation project which I will briefly describe here with an explanation of what methods and capabilities I have aimed to implement thus far and what I aim to implement in the future.

First, the collision/geometric handling. The program categorizes the geometry and the math of rigidbodies separately with a GeometricType abstract class. So far, the two options for Geometric Types are Circles and Polygons, which both use Separating Axis Theorem (SAT) to determine if and where a collision occurs between two finite bodies. All relevant values of the rigidbodies- shape, moment of inertia, center of mass, and mass- are either passed in through instantiation or are calculated using derived formulas for moment of inertia and the parallel axis theorem. Each collision treats itself as if it is the primary collision if others occur at the same time, and calculate impulses based on the equation v_n = -e * v_i along the normal of collision, where e is the coefficient of restitution. Collisions also take into account friction and the lack of true orthogonality preservation between these two orthogonal impulses. This change, wherein frictional and normal impulses are calculated to preserve both Coulomb's law of friction and the equation above described, have yet to be pushed to the remote repo, alongside changes later described.

Second, additional features. The simulation has multiple additional features like air resistance and universal gravitation. The implementation of these two mechanics takes the most basic form possible. With the former, the simulation does not take into account the complicated behaviors of fluids and instead applies a simple force calculated for each edge of the polygon at each point of that edge. For circles, this force is solely applied to the center of mass. With the latter, universal gravitation treats each rigidbody as a point mass at its center of mass, and no such algorithms (like the Barnes-Hut algorithm) have been implemented as of yet to improve the O(n^2) time complexity of universal gravitation calculations. One of these features is a player controller that works through reading keyboard input. Due to threading issues, it can experience issues when too many buttons are pressed at the same time.

Third, softbodies. The simulation features softbodies in 3 distinct forms using a combination of 3 main algorithms on a set of rigidbodies. These algorithms seek to emulate either pressure within the softbody from gas, or apply spring forces between each rigidbody and another location. These three algorithms are the pressure-spring approach, the spring-mass approach, and the shape-match approach. While the previous two use spring forces between connected rigidbodies and pressure within the softbody, the shape-match approach applies spring-like forces between each constituent rigidbody and its "ideal" position. These "ideal" positions are calculated by using the Least Squares approach to determine what angle the original shape best matches the most recent shape if rotated by that angle.

Fourth, joints. At the present, the only form of joint in the simulation are spring joints. I am in the process of actively developing distance-constraint joints, translational joints, pin joints, and weld joints so as to allow the creation of compound bodies rather than just rigidbodies or softbodies.

Fourth, the simulation makes use of either the Sweep-and-Prune algorithm or the Bounding-Volume-Hierarchy algorithm to optimize collision detection. In my own tests on my personal computer, this averages 50-90 fps with 2000 objects, even without optimization through multi-threading, which is also planned for the future. Other algorithms like quadtrees are planned to be implemented so that I can take advantage of the Barnes-Hut algorithm.

Fifth, the simulation categorizes properties into materials and physical constants that can be set and manipulated through calls to the Simulation or PhysicsObject or Hitbox classes, which act as the facades through which the programmer interfaces with the rest of the package. This obfuscates the lower level implementations of the program from the user while still allowing them to read and pass data from it.

What is not pushed to branches yet, but is a completed feature: coupling of friction and normal impulses, fixes to mouse joint behavior, distance constraint joint, pin joints, joints as solid objects that can be collided with.

I plan to implement the following in the future: quadtree optimizations, Barnes-Hut algorithm for universal gravitation between centers of mass, precise numerical calculations solely for gravitational calculations between extremely close rigidbodies, simplified CFD for air resistance, controllable muscle joints, improved visual scheme using a library other than Swing.

I have made progress on: different types of joints (welds and compound bodies).

Pipedreams: full on particle-based fluid simulation integration, optimization to run at 60 fps with 5000 objects.

Timeline: What is not pushed but is completed is planned to be pushed in the next update, alongside the finished implementation of welds and compound bodies by the end of January.


Demos Branch: This branch of the project acts as a branch to demo the features of the simulation. Typing in one of the following commands into the prompt interface will produce the following results -
"Demo1": Showcases the first demo with an immovable object and rigidbodies bouncing around. No additional features activated.
"Demo2": Showcases friction between sliding blocks.
"Demo3": Showcases the same as the "Demo1" but with air resistance turned on. Buoyancy is a derelict feature that used to be in this demo, but was removed.
"Demo4": Showcases universal gravitation between circle rigidbodies with orbits.
"Demo5": Showcases one-sided face objects and how they interact with a variety of rigidbodies with different properties and geometries.
"Demo6": Showcases springs being used to form ropes prior to springs being made solid in future updates.
"Demo7": Showcases pressure-spring and spring-mass softbodies and their ability to collide with other rigidbodies (later replaced with solid joint collisions).
"Demo8": Showcases the two types of shape-match softbodies and the repulsive effect between softbodies to give more realistic looking softbody collisions.
"Demo9": Showcases the player controller mechanics. Use WASD.
"Demo10": Showcases an optimization stress test with 2001 objects.
