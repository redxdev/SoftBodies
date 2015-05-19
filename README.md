# Soft Bodies

Simple soft body/rigid body interaction. The cloth is actually a set of springs rather than
cloth. Some code is based on Chris Hecker's 3d sample code.

# Building

This is built with libcinder and as such needs it installed. The visual studio solution is set up
as if the project is in a folder within the cinder distribution. For example, the solution
should be located at:

cinder_directory/projects/Physics/vc2013/Physics.sln

# Using the project

When launched, the only thing in the scene will be the springs/cloth. The panel at the top-left
can be used to change settings.

## World Settings

Click the plus next to the "gravity" option to show the current force for gravity. You can drag
the arrow around to change it.

Click the "Reset Simulation" button to reset the cloth and ball.

## Cloth Settings

By default, only the wireframe will show for the cloth. Check the "Draw Cloth" option to make the
cloth draw as a solid.

When the app starts, it will lock the four corners of the cloth to the world. Checking the
"Random locked points" box will lock a random selection of points to the world after resetting
the simulation (click the "reset simulation" button). Check the "no locked points" box to
not lock any points to the world (this won't be particularly interesting as the cloth will just
fall out of the world).

The spring constant and lengths can be changed via the settings panel.

## Ball Settings

Check the "Enable Ball" box to enable the ball.

# Algorithms

## World

All components of the world use a very simple euler integrator. There is the capacity for the
integrator to roll back the timestep if objects are penetrating, but that is not used at the
moment.

### Issues

The euler integrator isn't great, and might actually be the source of some of the other problems.

## Cloth

The cloth is simulated via a set of 10x10 nodes connected together by springs. Each node handles
its own simulation (technically the world handles the simulation, but it is done in such a way
that one node only applies force to itself relative to other nodes). Nodes have no mass but have
a position, velocity, and force. All nodes share a common spring constant and length which they
use to calculate forces. Nodes can be marked as locked to the world, at which point no force will
be applied to them.

### Issues

If the spring constant is set too high or the length too low, the springs tend to go crazy. If
left alone, the springs also tend to start spazzing out after a few minutes. Damping helps
this but does not get rid of it. This could probably be fixed by not using springs to simulate
cloth, or figuring out a better damping algorithm.

## Ball

The ball is simulated very simply with the euler integrator.

## Ball/Cloth Collision

Collisions are resolved in a _very_ simple manor, which is to apply a force relative to
the penetration distance between the ball and the cloth. Collisions are only found between
the individual nodes of the cloth and the ball, and as such large faces in the cloth will not
collide with the ball. Cloth nodes have collision forces applied with a multiplier as the
cloth's movement is damped and doesn't react as well otherwise.

### Issues

The ball doesn't have rotations working correctly at the moment, leading to some weird
movements at times. The cloth also doesn't react as realistically as it could primarily
due to the damping on the cloth's movement.